"""Draws the app icon (1024 px PNG): a generic gamepad in the app's blueprint style."""
import sys
from PIL import Image, ImageDraw, ImageFilter

S = 1024
out_path = sys.argv[1] if len(sys.argv) > 1 else "AppIcon-1024.png"


def layer():
    return Image.new("RGBA", (S, S), (0, 0, 0, 0))


def bez(p0, p1, p2, p3, n=40):
    return [((1-t)**3*p0[0] + 3*(1-t)**2*t*p1[0] + 3*(1-t)*t*t*p2[0] + t**3*p3[0],
             (1-t)**3*p0[1] + 3*(1-t)**2*t*p1[1] + 3*(1-t)*t*t*p2[1] + t**3*p3[1]) for t in [i/n for i in range(n+1)]]


segs = [((300, 112), (380, 96), (520, 96), (600, 112)), ((600, 112), (682, 104), (737, 120), (762, 160)),
        ((762, 160), (803, 222), (845, 342), (838, 432)), ((838, 432), (834, 500), (792, 532), (746, 522)),
        ((746, 522), (702, 512), (682, 432), (640, 410)), ((640, 410), (560, 386), (340, 386), (260, 410)),
        ((260, 410), (218, 432), (198, 512), (154, 522)), ((154, 522), (108, 532), (66, 500), (62, 432)),
        ((62, 432), (55, 342), (97, 222), (138, 160)), ((138, 160), (163, 120), (218, 104), (300, 112))]
pts = []
for s in segs:
    pts += bez(*s)
sc, ox, oy = 0.95, 512 - 450 * 0.95, 512 - 315 * 0.95
P = [(ox + x * sc, oy + y * sc) for x, y in pts]


def T(x, y):
    return (ox + x * sc, oy + y * sc)


base = Image.new("RGBA", (S, S), (14, 18, 25, 255))
grid = layer()
g = ImageDraw.Draw(grid)
for v in range(84, 964, 48):
    g.line((v, 0, v, S), fill=(110, 150, 180, 26), width=2)
    g.line((0, v, S, v), fill=(110, 150, 180, 26), width=2)
base.alpha_composite(grid)
fill = layer()
ImageDraw.Draw(fill).polygon(P, fill=(70, 220, 245, 30))
base.alpha_composite(fill)
glow = layer()
ImageDraw.Draw(glow).line(P + [P[0]], fill=(70, 220, 245, 170), width=28)
base.alpha_composite(glow.filter(ImageFilter.GaussianBlur(20)))

lines = layer()
d = ImageDraw.Draw(lines)
cyan, orange, violet = (80, 225, 248, 255), (255, 165, 75, 255), (200, 165, 255, 255)
d.line(P + [P[0]], fill=cyan, width=12, joint="curve")


def circ(x, y, r, col, w=10):
    cx, cy = T(x, y)
    rr = r * sc
    d.ellipse((cx - rr, cy - rr, cx + rr, cy + rr), outline=col, width=w)


for x in (334, 566):
    circ(x, 318, 46, cyan)
    circ(x, 318, 26, (80, 225, 248, 130), 6)
for (x, y) in [(688, 168), (734, 214), (688, 260), (642, 214)]:
    circ(x, y, 20, orange, 9)
cx, cy = T(212, 214)
a, l = 17 * sc, 52 * sc
d.rounded_rectangle((cx - a, cy - l, cx + a, cy + l), radius=8, outline=violet, width=9)
d.rounded_rectangle((cx - l, cy - a, cx + l, cy + a), radius=8, outline=violet, width=9)
x0, y0 = T(334, 122)
x1, y1 = T(566, 250)
d.rounded_rectangle((x0, y0, x1, y1), radius=26, outline=(170, 195, 220, 210), width=8)
base.alpha_composite(lines)

mask = Image.new("L", (S, S), 0)
ImageDraw.Draw(mask).rounded_rectangle((60, 60, 964, 964), radius=200, fill=255)
out = layer()
out.paste(base, (0, 0), mask)
out.save(out_path)
print("icon:", out_path)
