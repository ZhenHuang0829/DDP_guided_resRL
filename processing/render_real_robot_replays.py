"""Render measured hardware joint positions without simulating a controller.

Reproduction (NumPy, MuJoCo, Pillow and FFmpeg required)::

    python processing/render_real_robot_replays.py --model path/to/panda_nohand.xml --assets path/to/panda/assets

The eighteen released CSVs are sufficient for reproduction. ``--source`` is an
optional maintainer import of the archived 25-ms plotting export, not a
requirement for replay. No ROS, pandas, policy checkpoint or dynamics rollout
is used. Frame times are ordinary elapsed command times, never phase aligned.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "hardware" / "motion_replays"
WIDTH, HEIGHT, FPS = 1920, 1600, 30
VIEW_WIDTH, VIEW_HEIGHT = 640, 252
VIEW_TOP, ROW_HEIGHT = 214, 348
GRID_BOTTOM = VIEW_TOP + ROW_HEIGHT + VIEW_HEIGHT
PLOT_HEIGHT = 272
BG, TEXT, MUTED, GRID = "#08131F", "#F2F6FB", "#A5B6C8", "#27394B"
BLUE, CORAL, TEAL = "#70B8FF", "#FF8079", "#4AD4CB"
METHODS = (("pure_ddp_no_rl", "nominal_ddp", "Nominal DDP", BLUE),
           ("friction_ddp", "friction_ddp", "Friction DDP", "#F1BC57"),
           ("slotine_li", "slotine_li", "Slotine-Li", "#B59CFF"),
           ("ddp_gp", "ddp_gp", "DDP+GP", "#65D49B"),
           ("velocity_control", "joint_velocity", "Joint velocity control", "#E5A1D0"),
           ("ab05_residual_rl", "proposed", "Proposed", CORAL))
CASES = (
    ("circle_xz", "panda_ddp_circle_plan", "Circle in the XZ plane", -90),
    ("figure8_xz", "panda_ddp_figure8_xz_large", "Large figure-eight in the XZ plane", -90),
    ("ellipse_yz", "panda_ddp_ellipse_yz", "Ellipse in the YZ plane", 150),
)
CSV_FIELDS = ["time_s"] + [f"q{i}_rad" for i in range(1, 8)] + [f"q_ref{i}_rad" for i in range(1, 8)]
METRICS = (("ee_mm", "End-effector position error", "mm", 6.0),
           ("joint_deg", "Joint position error (RMS over 7 joints)", "deg", 6.0))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=True) + "\n", encoding="utf-8", newline="\n")


def source_names_only(value):
    """Keep archive identifiers and hashes without private workspace paths."""
    if isinstance(value, dict):
        return {("name" if k == "path" else k): (Path(v).name if k == "path" else source_names_only(v))
                for k, v in value.items()}
    if isinstance(value, list):
        return [source_names_only(v) for v in value]
    return value


def font(size, bold=False, override=None):
    candidates = [override] if override else []
    system_fonts = Path(os.environ.get("WINDIR", "")) / "Fonts"
    candidates.append(system_fonts / ("arialbd.ttf" if bold else "arial.ttf"))
    for candidate in candidates:
        if candidate and Path(candidate).is_file():
            return ImageFont.truetype(str(candidate), size)
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)
    except OSError:
        return ImageFont.load_default(size=size)


def make_model(path, assets_dir=None):
    """Create a display-only scene; joints are set from measurements each frame."""
    tree = ET.parse(path)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is not None:
        for attr in ("meshdir", "texturedir"):
            directory = compiler.get(attr)
            if directory:
                resolved = assets_dir.resolve() if attr == "meshdir" and assets_dir else (path.parent / directory).resolve()
                if not resolved.is_dir():
                    raise ValueError(f"Missing model assets. Supply --assets with the Panda mesh directory: {directory}")
                compiler.set(attr, str(resolved))
    for tag in ("actuator", "keyframe", "visual"):
        for element in list(root.findall(tag)):
            root.remove(element)
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(VIEW_WIDTH), offheight=str(VIEW_HEIGHT))
    ET.SubElement(visual, "quality", shadowsize="2048", offsamples="4")
    ET.SubElement(visual, "headlight", diffuse=".55 .58 .64", ambient=".34 .37 .42", specular=".12 .12 .12")
    ET.SubElement(visual, "rgba", haze=".031 .075 .122 1")
    asset = root.find("asset")
    ET.SubElement(asset, "texture", name="replay_sky", type="skybox", builtin="gradient",
                  rgb1=".031 .075 .122", rgb2=".065 .11 .17", width="512", height="3072")
    ET.SubElement(asset, "material", name="replay_floor", rgba=".065 .11 .17 1",
                  reflectance=".02", specular=".15", shininess=".2")
    world = root.find("worldbody")
    ET.SubElement(world, "geom", name="replay_floor", type="plane", size="3 3 .1",
                  pos="0 0 -.012", material="replay_floor", contype="0", conaffinity="0")
    ET.SubElement(world, "light", pos="1 -2 3", dir="-.3 .6 -1", diffuse=".75 .8 .9", castshadow="true")
    ET.SubElement(world, "light", pos="-1 2 2", dir=".2 -.6 -1", diffuse=".5 .6 .7", castshadow="false")
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    if model.nq != 7:
        raise ValueError("Expected the seven-joint Panda model without a gripper")
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "panda_link8_site")
    if site < 0:
        raise ValueError("The model must contain panda_link8_site (the experimental reference frame)")
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(7)]
    if names != [f"joint{i}" for i in range(1, 8)]:
        raise ValueError(f"Unexpected joint ordering: {names}")
    return model, mujoco.MjData(model), site


def positions(model, data, site, q):
    result = np.empty((len(q), 3))
    for i, value in enumerate(q):
        data.qpos[:] = value
        mujoco.mj_forward(model, data)
        result[i] = data.site_xpos[site]
    return result


def arrays(rows):
    values = np.array([[float(row[name]) for name in CSV_FIELDS] for row in rows])
    if not np.isfinite(values).all() or np.any(np.diff(values[:, 0]) <= 0):
        raise ValueError("Expected finite values and strictly increasing measured timestamps")
    return values[:, 0], values[:, 1:8], values[:, 8:15]


def import_archive(source, extra_sources, summary, evidence, output, model, data, site, model_path):
    """Export single measured runs, retaining original timestamps and values."""
    wanted = {case[1] for case in CASES}
    records = {}
    for archive in [source, *extra_sources]:
        opener = gzip.open if archive.suffix == ".gz" else open
        with opener(archive, "rt", encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                if row["cohort"] == "regular" and row["traj"] in wanted and row["method"] in {m[0] for m in METHODS}:
                    records.setdefault((row["traj"], row["method"], row["run"]), []).append(row)
    summaries = []
    if summary:
        with summary.open(encoding="utf-8", newline="") as stream:
            summaries = list(csv.DictReader(stream))
    manifest = {
        "schema_version": 2,
        "description": "Kinematic reconstruction of recorded real-robot joint measurements, not camera footage or a new dynamics simulation.",
        "source_archive": {"name": source.name, "sha256": digest(source)},
        "additional_source_archives": [{"name": p.name, "sha256": digest(p)} for p in extra_sources],
        "source_model": {"name": model_path.name, "sha256": digest(model_path), "end_frame": "panda_link8", "render_site": "panda_link8_site"},
        "selection": "Three predetermined geometric examples (XZ circle, large XZ figure-eight, YZ ellipse); not selected by improvement or ranking. First valid run in lexical run order; no averaging across repeats.",
        "sampling": "Approximately 25 ms: the source plotting export selects measured samples at or after each 25-ms target time. Released CSVs retain their actual timestamps and unmodified q and q_ref values.",
        "reference": "Each run's recorded command_plan, interpolated at its measured joint timestamps by the original analysis. Historical reference timing may differ across methods.",
        "synchronization": "Shared elapsed command time and the intersection of recorded intervals. No phase alignment, time warping, extrapolation, error magnification or motion scaling.",
        "interpolation": "Linear interpolation of measured joint angles and the corresponding recorded reference angles at 30 video frames/s. MuJoCo forward kinematics only; no mj_step or control rollout.",
        "statistics": "Per-run means/RMS of the native released samples within the displayed replay window. They are illustrative replay-window metrics, not full-rate paper aggregates. No statistics are computed from rendered video frames.",
        "payload_note": "No payload claim is attached to these clips: the historical source has cohort-level evidence but no independent per-run mass label.",
        "cases": [],
    }
    if summary:
        manifest["source_summary"] = {"name": summary.name, "sha256": digest(summary)}
    if evidence:
        manifest["additional_source_evidence"] = source_names_only(json.loads(evidence.read_text(encoding="utf-8")))
    (output / "data").mkdir(parents=True, exist_ok=True)
    for key, trajectory, title, azimuth in CASES:
        case = {"id": key, "trajectory_id": trajectory, "title": title, "cohort": "Training", "camera_azimuth_deg": azimuth, "runs": []}
        for source_method, method, label, _ in METHODS:
            choices = sorted(run for (traj, name, run) in records if traj == trajectory and name == source_method)
            selected = None
            for run in choices:
                rows = sorted(records[trajectory, source_method, run], key=lambda row: float(row["time_s"]))
                try:
                    t, q, qr = arrays(rows)
                    if len(t) < 350 or t[-1] - t[0] < 9:
                        continue
                except ValueError:
                    continue
                selected = (run, rows, t, q, qr)
                break
            if selected is None:
                raise ValueError(f"No valid measured run for {trajectory}: {source_method}")
            run, rows, t, q, qr = selected
            xyz, ref = positions(model, data, site, q), positions(model, data, site, qr)
            recorded_xyz = np.array([[float(r[f"{axis}_m"]) for axis in "xyz"] for r in rows])
            recorded_ref = np.array([[float(r[f"{axis}_ref_m"]) for axis in "xyz"] for r in rows])
            delta = float(max(np.max(np.abs(xyz - recorded_xyz)), np.max(np.abs(ref - recorded_ref))))
            error_delta = float(np.max(np.abs(np.linalg.norm(xyz-ref, axis=1)*1000 - [float(r["ee_mm"]) for r in rows])))
            if delta > 1e-10 or error_delta > 1e-7:
                raise ValueError(f"Display model and archived kinematics differ: {delta} m, {error_delta} mm")
            relative = f"data/{key}_{method}.csv"
            target = output / relative
            with target.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(CSV_FIELDS)
                writer.writerows([[format(float(r[name]), ".17g") for name in CSV_FIELDS] for r in rows])
            payload = [r for r in summaries if r["traj"] == trajectory and r["method"] == source_method and r["cohort"] == "regular"]
            case["runs"].append({
                "method": method, "label": label, "source_method": source_method,
                "source_run": run, "available_runs": len(choices), "selection": "First valid run in lexical run order; no averaging or selection by tracking performance.",
                "file": relative, "sha256": digest(target), "samples": len(t),
                "recorded_interval_s": [float(t[0]), float(t[-1])], "median_sample_spacing_s": float(np.median(np.diff(t))),
                "source_geometry_max_difference_m": delta, "source_ee_error_max_difference_mm": error_delta,
                "payload_evidence": payload[0].get("payload_evidence", "Not independently verified") if payload else "Not independently verified",
            })
        manifest["cases"].append(case)
    save_json(output / "provenance.json", manifest)
    return manifest


def load_case(case, output, model, data, site):
    if [r["method"] for r in case["runs"]] != [m[1] for m in METHODS]:
        raise ValueError("Replay must contain all six Figure 4 methods in the declared panel order")
    runs = []
    for record in case["runs"]:
        path = output / record["file"]
        if digest(path) != record["sha256"]:
            raise ValueError(f"Data hash mismatch: {record['file']}")
        with path.open(encoding="utf-8", newline="") as stream:
            t, q, qr = arrays(list(csv.DictReader(stream)))
        xyz, ref = positions(model, data, site, q), positions(model, data, site, qr)
        runs.append(dict(record=record, t=t, q=q, qr=qr, xyz=xyz, ref=ref,
                         ee_mm=np.linalg.norm(xyz-ref, axis=1)*1000,
                         joint_deg=np.rad2deg(np.sqrt(np.mean((q-qr)**2, axis=1)))))
    lo, hi = max(r["t"][0] for r in runs), min(r["t"][-1] for r in runs)
    frames = lo + np.arange(int(math.floor((hi-lo)*FPS))+1) / FPS
    for run in runs:
        run["frame_q"] = np.column_stack([np.interp(frames, run["t"], run["q"][:, j]) for j in range(7)])
        run["frame_qr"] = np.column_stack([np.interp(frames, run["t"], run["qr"][:, j]) for j in range(7)])
        run["frame_xyz"] = positions(model, data, site, run["frame_q"])
        run["frame_ref"] = positions(model, data, site, run["frame_qr"])
        mask = (run["t"] >= frames[0]) & (run["t"] <= frames[-1])
        run["window_mask"] = mask
        run["statistics"] = {"native_samples": int(mask.sum()),
                             "ee_mean_mm": float(run["ee_mm"][mask].mean()),
                             "ee_rms_mm": float(np.sqrt(np.mean(run["ee_mm"][mask]**2))),
                             "ee_max_mm": float(run["ee_mm"][mask].max()),
                             "joint_global_rms_deg": float(np.sqrt(np.mean(run["joint_deg"][mask]**2)))}
        for metric, _, _, ymax in METRICS:
            if float(run[metric].max()) >= ymax:
                raise ValueError(f"Fixed axis would crop {case['id']}, {metric}")
    return runs, frames, [float(lo), float(hi)]


def rgba(hex_color, alpha=1.):
    return [int(hex_color[i:i+2], 16)/255 for i in (1, 3, 5)] + [alpha]


def segment(scene, p1, p2, radius, color):
    if np.linalg.norm(p1-p2) < 1e-10:
        return
    if scene.ngeom >= scene.maxgeom:
        raise ValueError("Trajectory exceeds MuJoCo scene capacity")
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), np.asarray(color, dtype=np.float32))
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, p1, p2)
    geom.category = mujoco.mjtCatBit.mjCAT_DECOR
    scene.ngeom += 1


def robot_frame(renderer, model, data, run, index, azimuth, color):
    data.qpos[:] = run["frame_q"][index]
    mujoco.mj_forward(model, data)
    camera = mujoco.MjvCamera()
    camera.lookat[:] = [.26, 0, .40]
    camera.distance, camera.azimuth, camera.elevation = 1.30, azimuth, -12
    if azimuth == 150:
        camera.lookat[:] = [.17, 0, .39]
        camera.distance = 1.28
    option = mujoco.MjvOption()
    option.geomgroup[3] = 0
    option.sitegroup[:] = 0
    renderer.update_scene(data, camera, scene_option=option)
    # Full recorded reference is dashed; measured movement is only the elapsed trail.
    ref = run["ref"]
    for j in range(0, len(ref)-2, 4):
        segment(renderer.scene, ref[j], ref[j+2], .0014, rgba(TEAL, .8))
    actual = run["frame_xyz"]
    for j in range(0, index):
        segment(renderer.scene, actual[j], actual[j+1], .0018, rgba(color))
    return Image.fromarray(renderer.render())


def backdrop(case, runs, fonts):
    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    draw.rounded_rectangle((28, 18, 923, 63), radius=10, fill="#123044")
    draw.text((44, 25), "REAL-ROBOT DATA  |  MuJoCo kinematic replay", font=fonts[26], fill="#A8E5E5")
    draw.text((1710, 27), "1x playback", font=fonts[26], fill=MUTED)
    draw.text((28, 76), case["title"], font=fonts[38], fill=TEXT)
    draw.text((28, 124), "Recorded trials  /  same elapsed command time  /  each method's own recorded reference", font=fonts[22], fill=MUTED)
    draw.line((1470, 137, 1510, 137), fill=TEAL, width=3)
    draw.text((1522, 123), "Recorded reference (dashed)", font=fonts[22], fill=TEAL)
    for i, (run, method) in enumerate(zip(runs, METHODS)):
        x, y = (i % 3)*VIEW_WIDTH + 28, (i // 3)*ROW_HEIGHT
        draw.rounded_rectangle((x, y+160, x+5, y+209), radius=2, fill=method[3])
        draw.text((x+17, y+156), method[2], font=fonts[30], fill=method[3])
        draw.text((x+17, y+192), f"Replay-window mean EE error  {run['statistics']['ee_mean_mm']:.3f} mm", font=fonts[20], fill=TEXT)
    for x in (640, 1280):
        draw.line((x, 156, x, GRID_BOTTOM), fill=GRID, width=2)
    divider_y = VIEW_TOP + VIEW_HEIGHT + 16
    draw.line((28, divider_y, 1892, divider_y), fill=GRID, width=2)
    for i, method in enumerate(METHODS):
        x = 28 + i*316
        draw.line((x, GRID_BOTTOM+30, x+28, GRID_BOTTOM+30), fill=method[3], width=4)
        draw.text((x+39, GRID_BOTTOM+16), method[2], font=fonts[22], fill=method[3])
    plots = []
    for index, (metric, title, units, ymax) in enumerate(METRICS):
        top = GRID_BOTTOM + 60 + (PLOT_HEIGHT+72)*index
        box = (88, top+42, 1870, top+42+PLOT_HEIGHT)
        x0, y0, x1, y1 = box
        draw.text((28, top-2), title + "  /  " + units, font=fonts[26], fill=TEXT)
        for value in (0, ymax/2, ymax):
            yy = y1-(y1-y0)*value/ymax
            draw.line((x0, yy, x1, yy), fill=GRID, width=1)
            label = f"{value:g}"
            draw.text((x0-12-draw.textlength(label, font=fonts[22]), yy-13), label, font=fonts[22], fill=MUTED)
        for tick in (0, 2, 4, 6, 8, 10):
            xx = x0+(x1-x0)*tick/10
            draw.line((xx, y0, xx, y1), fill=GRID, width=1)
            if index == 1:
                label = str(tick)
                draw.text((xx-draw.textlength(label, font=fonts[22])/2, y1+6), label, font=fonts[22], fill=MUTED)
        draw.line((x0, y0, x0, y1, x1, y1), fill="#597085", width=1)
        points = []
        for run, method in zip(runs, METHODS):
            faded = tuple(round(255*(.26*c + .74*b)) for c, b in zip(rgba(method[3])[:3], rgba(BG)[:3]))
            xy = [(x0+(x1-x0)*t/10, y1-(y1-y0)*v/ymax) for t, v in zip(run["t"], run[metric])]
            draw.line(xy, fill=faded, width=2)
            points.append(xy)
        plots.append((box, points, top))
    draw.text((800, HEIGHT-30), "Elapsed command time / s", font=fonts[22], fill=MUTED)
    draw.text((28, HEIGHT-30), "Motion and errors at true scale", font=fonts[20], fill=MUTED)
    return image, plots


def compose(renderer, model, data, case, runs, frames, index, background, plots, fonts):
    image = background.copy()
    for i, (run, method) in enumerate(zip(runs, METHODS)):
        image.paste(robot_frame(renderer, model, data, run, index, case["camera_azimuth_deg"], method[3]), ((i % 3)*VIEW_WIDTH, VIEW_TOP + (i // 3)*ROW_HEIGHT))
    draw = ImageDraw.Draw(image)
    time = float(frames[index])
    draw.text((1710, 81), f"t = {time:05.2f} s", font=fonts[26], fill=TEXT)
    for (metric, _, units, ymax), (box, point_sets, top) in zip(METRICS, plots):
        x0, y0, x1, y1 = box
        xx = x0+(x1-x0)*time/10
        for i, (run, points, method) in enumerate(zip(runs, point_sets, METHODS)):
            n = int(np.searchsorted(run["t"], time, side="right"))
            value = float(np.interp(time, run["t"], run[metric]))
            yy = y1-(y1-y0)*value/ymax
            line = points[:n] + [(xx, yy)]
            if len(line) > 1:
                draw.line(line, fill=method[3], width=3)
            draw.ellipse((xx-4, yy-4, xx+4, yy+4), fill=method[3])
            draw.text((750+i*190, top-2), f"{value:.3f} {units}", font=fonts[22], fill=method[3])
        draw.line((xx, y0, xx, y1), fill="#B6C3D0", width=1)
    return image


def probe(path):
    return json.loads(subprocess.check_output(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
                                               "stream=codec_name,width,height,r_frame_rate,nb_frames:format=duration,size", "-of", "json", str(path)]))


def render_case(case, output, model, data, site, fonts, preview):
    runs, times, recorded_window = load_case(case, output, model, data, site)
    background, plots = backdrop(case, runs, fonts)
    media = output / "media"
    media.mkdir(exist_ok=True)
    poster_index = int(np.argmin(np.abs(times-4.0)))
    with mujoco.Renderer(model, VIEW_HEIGHT, VIEW_WIDTH) as renderer:
        poster = compose(renderer, model, data, case, runs, times, poster_index, background, plots, fonts)
        poster.save(media / f"{case['id']}_poster.png")
        if preview:
            for t in (1.0, 7.5):
                i = int(np.argmin(np.abs(times-t)))
                compose(renderer, model, data, case, runs, times, i, background, plots, fonts).save(media / f"{case['id']}_preview_{t:g}s.png")
            print(f"Previewed {case['id']}", flush=True)
            return None
        video = media / f"{case['id']}.mp4"
        command = ["ffmpeg", "-y", "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}",
                   "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-crf", "23", "-preset", "medium", "-threads", "4",
                   "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video)]
        process = subprocess.Popen(command, stdin=subprocess.PIPE)
        try:
            for i in range(len(times)):
                frame = compose(renderer, model, data, case, runs, times, i, background, plots, fonts)
                process.stdin.write(frame.tobytes())
                if i % 90 == 0:
                    print(f"{case['id']}: {i}/{len(times)} frames", flush=True)
        finally:
            process.stdin.close()
        if process.wait():
            raise RuntimeError("FFmpeg encoding failed")
    gif = media / f"{case['id']}.gif"
    # Rectangle differences keep large static backgrounds compact without shrinking text further.
    filters = "fps=10,scale=960:-1:flags=lanczos,split[a][b];[a]palettegen=max_colors=192:stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3:diff_mode=rectangle"
    subprocess.run(["ffmpeg", "-y", "-v", "error", "-i", str(video), "-filter_complex", filters, "-loop", "0", str(gif)], check=True)
    subprocess.run(["ffmpeg", "-v", "error", "-i", str(video), "-f", "null", "-"], check=True)
    info = probe(video)
    stream = info["streams"][0]
    if int(stream["nb_frames"]) != len(times) or stream["r_frame_rate"] != "30/1":
        raise ValueError("Encoded frame sequence differs from requested playback")
    return {"recorded_common_interval_s": recorded_window, "replayed_sample_interval_s": [float(times[0]), float(times[-1])],
            "frame_count": len(times), "fps": FPS, "playback_speed": 1.0,
            "encoded_duration_s": float(info["format"]["duration"]),
            "timing_note": "Video frame n shows elapsed command time start + n/30. The last frame is held for one encoder frame period; there is no time scaling.",
            "metric_curve_interpolation": "Native per-sample FK errors are linearly interpolated for the animated curve cursor. Robot poses use linear joint-angle interpolation; nonlinear FK can produce a small interpolation difference between the drawn robot and cursor value.",
            "axis_limits": {"time_s": [0, 10], "ee_error_mm": [0, 6], "joint_error_deg": [0, 6]},
            "statistics_by_method": {r["record"]["method"]: r["statistics"] for r in runs},
            "files": {p.name: {"sha256": digest(p), "bytes": p.stat().st_size} for p in (video, gif, media / f"{case['id']}_poster.png")},
            "verification": "Decoded full MP4 without errors; frame count and frame rate match the measured-time replay."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, type=Path, help="Panda no-hand MJCF with panda_link8_site and its referenced mesh assets")
    parser.add_argument("--assets", type=Path, help="External Panda mesh directory, overriding the MJCF meshdir")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--source", type=Path, help="Optional maintainer import: original processed plot_series_25ms.csv.gz")
    parser.add_argument("--extra-source", action="append", type=Path, default=[], help="Additional measured-run export with the same columns")
    parser.add_argument("--source-evidence", type=Path, help="Optional additional source provenance JSON")
    parser.add_argument("--source-summary", type=Path, help="Optional original per_trajectory.csv for payload provenance")
    parser.add_argument("--preview", action="store_true", help="Only produce three still previews per case")
    parser.add_argument("--case", choices=[case[0] for case in CASES], action="append")
    parser.add_argument("--font", type=Path)
    args = parser.parse_args()
    if (args.extra_source or args.source_evidence) and not args.source:
        parser.error("--extra-source and --source-evidence require --source")
    args.model = args.model.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    for executable in ("ffmpeg", "ffprobe"):
        if not args.preview and shutil.which(executable) is None:
            parser.error(f"{executable} is required on PATH")
    model, data, site = make_model(args.model, args.assets)
    asset_manifest = args.model.parent / "mesh_manifest.json"
    if asset_manifest.exists():
        hashes = json.loads(asset_manifest.read_text(encoding="utf-8"))
        asset_root = args.assets or args.model.parent / "assets"
        for filename, record in hashes["external_meshes"].items():
            if digest(asset_root / filename) != record["sha256"]:
                raise ValueError(f"Mesh differs from the validated display asset: {filename}")
    if args.source:
        manifest = import_archive(args.source, args.extra_source, args.source_summary, args.source_evidence,
                                  args.output, model, data, site, args.model)
    else:
        manifest = json.loads((args.output / "provenance.json").read_text(encoding="utf-8"))
        if digest(args.model) != manifest["source_model"]["sha256"]:
            raise ValueError("Model differs from the validated source model; validate its kinematics before substitution")
    fonts = {size: font(size, bold=size in (30, 38), override=args.font) for size in (18, 20, 22, 26, 30, 38)}
    for case in manifest["cases"]:
        if args.case and case["id"] not in args.case:
            continue
        rendered = render_case(case, args.output, model, data, site, fonts, args.preview)
        if rendered:
            case["render"] = rendered
            manifest["renderer"] = {"script": Path(__file__).name, "sha256": digest(__file__), "mujoco_version": mujoco.__version__,
                                    "dimensions": [WIDTH, HEIGHT], "gif_dimensions": [960, HEIGHT//2], "gif_fps": 10,
                                    "error_plot_height_px": PLOT_HEIGHT,
                                    "panel_layout": "Three columns by two rows; Figure 4 method order",
                                    "method_colors": {m[1]: m[3] for m in METHODS},
                                    "font_request": args.font.name if args.font else "Platform Arial/DejaVu Sans fallback"}
            manifest["source_model"]["bundled_file"] = "model/panda_nohand.xml"
            manifest["source_model"]["external_mesh_manifest"] = "model/mesh_manifest.json"
            manifest["source_model"]["upstream"] = "https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda"
            save_json(args.output / "provenance.json", manifest)


if __name__ == "__main__":
    main()
