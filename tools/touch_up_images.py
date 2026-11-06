"""Touch up the photos and screenshots used on the website.

Writes edited copies to images/ and leaves the originals where they are.
Every edit is global and non-generative: cropping junk borders, white
balance, local contrast, light denoising and an unsharp mask. The cover
photo and the four smallest images also get a 2x super-resolution pass with EDSR (OpenCV
dnn_superres, model EDSR_x2.pb from github.com/Saafke/EDSR_Tensorflow),
a regression model that sharpens edges rather than inventing texture.
Nothing in any picture is added, removed or moved. The portrait only
gets light sharpening, no smoothing.

    python tools/touch_up_images.py path/to/EDSR_x2.pb [output name ...]

With output names (e.g. portrait.jpg), only those files are redone.

Needs numpy and opencv-contrib-python(-headless).
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "images"


def load(name):
    """Read as 8-bit BGR; transparent pixels are flattened onto white."""
    img = cv2.imread(str(ROOT / name), cv2.IMREAD_UNCHANGED)
    if img.ndim == 3 and img.shape[2] == 4:
        a = img[:, :, 3:4].astype(np.float32) / 255
        img = (img[:, :, :3].astype(np.float32) * a + 255 * (1 - a)).round().astype(np.uint8)
    return img


def gray_world(img, strength=1.0):
    """Neutralise a colour cast by pulling the channel means together."""
    f = img.astype(np.float32)
    means = f.reshape(-1, 3).mean(axis=0)
    gains = 1 + strength * (means.mean() / means - 1)
    return np.clip(f * gains, 0, 255).astype(np.uint8)


def local_contrast(img, clip=1.4, tiles=8):
    """CLAHE on lightness only, so colours don't shift."""
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=clip, tileGridSize=(tiles, tiles)).apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def denoise(img, h=4):
    return cv2.fastNlMeansDenoisingColored(img, None, h, h, 7, 21)


def unsharp(img, sigma=1.2, amount=0.5, threshold=2):
    blur = cv2.GaussianBlur(img, (0, 0), sigma)
    diff = img.astype(np.float32) - blur.astype(np.float32)
    diff[np.abs(diff) <= threshold] = 0
    return np.clip(img.astype(np.float32) + amount * diff, 0, 255).astype(np.uint8)


def trim_white(img, pad=14, tol=240):
    """Crop away uniform white margins, keeping a small even border."""
    ys, xs = np.where((img < tol).any(axis=2))
    y0, y1 = max(ys.min() - pad, 0), min(ys.max() + pad + 1, img.shape[0])
    x0, x1 = max(xs.min() - pad, 0), min(xs.max() + pad + 1, img.shape[1])
    return img[y0:y1, x0:x1]


def resize_width(img, width):
    h = round(img.shape[0] * width / img.shape[1])
    interp = cv2.INTER_AREA if width < img.shape[1] else cv2.INTER_LANCZOS4
    return cv2.resize(img, (width, h), interpolation=interp)


def save_jpg(img, name, quality=87):
    cv2.imwrite(str(OUT / name), img, [cv2.IMWRITE_JPEG_QUALITY, quality,
                                       cv2.IMWRITE_JPEG_PROGRESSIVE, 1, cv2.IMWRITE_JPEG_OPTIMIZE, 1])
    print(f"wrote images/{name} {img.shape[1]}x{img.shape[0]}", flush=True)


def save_png(img, name):
    cv2.imwrite(str(OUT / name), img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    print(f"wrote images/{name} {img.shape[1]}x{img.shape[0]}", flush=True)


def recipes(sr):
    """Output name -> function that writes it. sr is the 2x EDSR upscaler."""
    def purdue():   # crop the tan frame and the cut-off logo; 2x EDSR so the full-width cover holds up on retina
        img = sr().upsample(load("purdue_arch.jpg")[14:885, 0:1328])
        img = unsharp(local_contrast(img, clip=1.2), sigma=1.2, amount=0.25)
        save_jpg(resize_width(img, 2400), "purdue_arch.jpg", 82)

    def portrait():   # light sharpening only: no smoothing, nothing about the face changes
        save_jpg(unsharp(load("image_f58a52.png"), sigma=0.8, amount=0.35, threshold=2), "portrait.jpg", 92)

    def phone_photo(src):   # light denoise, a little local contrast, sharpening
        return lambda: save_jpg(unsharp(local_contrast(denoise(load(src), h=3), clip=1.25), amount=0.45), src, 88)

    def gel():   # remove the green cast, denoise, 2x EDSR, then local contrast
        img = denoise(gray_world(load("image_05119d.png"), strength=0.7), h=5)
        img = unsharp(local_contrast(sr().upsample(img), clip=1.4), sigma=1.4, amount=0.45)
        save_jpg(resize_width(img, 1500), "gel_cantilever.jpg", 88)

    def force():   # screenshot of the MATLAB plots: 2x EDSR so lines and labels are crisper
        save_png(unsharp(sr().upsample(load("image_05153e.png")), sigma=1.0, amount=0.35), "force_vs_time.png")

    def cad(src, dst):   # trim the white margins, 2x EDSR, light sharpening
        return lambda: save_png(unsharp(trim_white(sr().upsample(trim_white(load(src)))), sigma=1.0, amount=0.3), dst)

    def dashboard():   # only the thin coloured frame from the screenshot tool is cropped
        save_png(load("Caterpillar.png")[2:-2, 2:-2], "caterpillar_dashboard.png")

    return {
        "purdue_arch.jpg": purdue,
        "portrait.jpg": portrait,
        "orthosis_built.jpg": phone_photo("orthosis_built.jpg"),
        "keyence_rig.jpg": phone_photo("keyence_rig.jpg"),
        "gel_cantilever.jpg": gel,
        "force_vs_time.png": force,
        "valve_actuator_cad.png": cad("image_058215.png", "valve_actuator_cad.png"),
        "valve_connection_cad.png": cad("image_05825b.png", "valve_connection_cad.png"),
        "caterpillar_dashboard.png": dashboard,
    }


def main(model_path, only):
    OUT.mkdir(exist_ok=True)
    model = []

    def sr():   # load the model on first use, so cheap outputs don't pay for it
        if not model:
            m = cv2.dnn_superres.DnnSuperResImpl_create()
            m.readModel(model_path)
            m.setModel("edsr", 2)
            model.append(m)
        return model[0]

    todo = recipes(sr)
    unknown = set(only) - set(todo)
    if unknown:
        sys.exit(f"unknown output(s): {', '.join(sorted(unknown))}; choose from {', '.join(todo)}")
    for name, make in todo.items():
        if not only or name in only:
            make()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2:])
