#!/usr/bin/env python3
"""
Grid macros simulator: try the macro half of the grid on a computer, no Pi, grid or rack needed.

It runs the real macro engine (src/macros/) with stand-ins for the rig: a 16n (8 faders), an EC4
(4 encoders), the DualSense script's slice / freeze state and Teletype's half of the grid. Snapshots go to
their own file (default ~/.config/dualsense-eurorack-midi/grid_macros_sim.json, same format as the real
grid_macros.json), so you can design banks here and look at the result.

    python3 src/grid_macros_sim.py            # then open http://localhost:8301
    python3 src/grid_macros_sim.py --port 8400 --store /tmp/macros.json
"""
import argparse
import json
import os
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import live_config  # noqa: E402
from macros.engine import MacroEngine  # noqa: E402
from macros.providers import MidiState, Provider, TeletypeLeds  # noqa: E402
from macros.store import BANKS, SLOTS, MacroStore  # noqa: E402

FADER_CH, FADER_CC0 = 16, 80          # 16n as set up here: ch 16, CC 80 …
EC4_CH, EC4_CC0 = 1, 1


class SimDualSense(Provider):
    """The DualSense script's state as the real snapshot socket would report it."""
    name = "dualsense"

    def __init__(self):
        self.state = {"block": 0, "section": 0, "fine": 0, "freeze": {"left": False, "right": False}}

    def slice(self):
        s = self.state
        return s["block"] * 128 + min(s["section"] * 16 + s["fine"], 127)

    def capture(self):
        d = json.loads(json.dumps(self.state))
        d["slice"] = self.slice()
        return d

    def restore(self, data, glide, duration, now, jump_at):
        glide.jump(("dualsense",), data, duration, now, self._apply, at=jump_at)

    def _apply(self, data):
        for k in ("block", "section", "fine"):
            if k in data:
                self.state[k] = int(data[k])
        if isinstance(data.get("freeze"), dict):
            self.state["freeze"] = {"left": bool(data["freeze"].get("left")),
                                    "right": bool(data["freeze"].get("right"))}

    def describe(self, data):
        fr = [k for k in ("left", "right") if (data.get("freeze") or {}).get(k)]
        return f"slice {data.get('slice', '?')}" + (f" · frozen {'+'.join(fr)}" if fr else "")


class Sim:
    def __init__(self, store_path):
        self.lock = threading.Lock()
        self.store = MacroStore(store_path)
        if not os.path.exists(store_path):
            self.store.save()
        s = self.store.settings
        s["midi_sources"] = ["16n", "EC4"]
        self.out_log = []
        self.faders = [0] * 8                  # physical positions
        self.knobs = [0] * 4                   # EC4 encoder values (they follow feedback)
        self.midi = MidiState(self._out, s, feedback=self._feedback, arm_pickup=lambda e: None)
        self.ds = SimDualSense()
        self.tt = TeletypeLeds()
        self.tt_leds = [0] * 64
        for i in (0, 9, 18, 27, 36, 45, 54, 63):
            self.tt_leds[i] = 6
        self.tt.update(self.tt_leds)
        self.engine = MacroEngine(self.store, {"midi": self.midi, "dualsense": self.ds, "teletype": self.tt},
                                  log=self._log)
        self.log_lines = []
        # give the stand-ins a starting point, so the first save has something in it
        for i in range(8):
            self.move_fader(i, 20 + i * 10)
        for i in range(4):
            self.turn_knob(i, 64)

    def _log(self, line):
        self.log_lines = (self.log_lines + [time.strftime("%H:%M:%S ") + line])[-8:]

    def _out(self, msg):
        self.out_log = (self.out_log + [msg])[-10:]

    def _feedback(self, src, msg):
        if src == "EC4" and (msg[0] & 0x0F) + 1 == EC4_CH and EC4_CC0 <= msg[1] < EC4_CC0 + 4:
            self.knobs[msg[1] - EC4_CC0] = msg[2]

    # stand-in controllers
    def move_fader(self, i, v):
        self.faders[i] = v
        self.midi.observe("16n sim", [0xB0 | (FADER_CH - 1), FADER_CC0 + i, v])

    def turn_knob(self, i, v):
        self.knobs[i] = v
        self.midi.observe("EC4 sim", [0xB0 | (EC4_CH - 1), EC4_CC0 + i, v])

    def tick_forever(self):
        while True:
            with self.lock:
                self.engine.tick()
            time.sleep(0.02)

    def state(self):
        e = self.engine
        bank = e.bank
        slots = []
        for s in range(SLOTS):
            snap = self.store.slot(bank, s)
            info = {"saved": snap is not None, "slew": self.store.slew(bank, s),
                    "glide": self.store.glide_time(bank, s), "active": e.active.get(bank) == s}
            if snap:
                info["when"] = time.strftime("%H:%M:%S", time.localtime(snap.get("saved_at", 0)))
                info["parts"] = [f"{n}: {e.providers[n].describe(d)}" for n, d in (snap.get("state") or {}).items()
                                 if n in e.providers]
            slots.append(info)
        params = [self.midi.values.get(("cc", FADER_CH, FADER_CC0 + i)) for i in range(8)]
        armed = [self.engine.pickup.armed(("cc", FADER_CH, FADER_CC0 + i)) for i in range(8)]
        gl = [e.glide.gliding(("cc", FADER_CH, FADER_CC0 + i)) for i in range(8)]
        return {
            "leds": e.leds(), "tt": self.tt_leds, "bank": bank, "slots": slots,
            "banks_used": [self.store.bank_used(b) for b in range(BANKS)],
            "faders": self.faders, "params": params, "armed": armed, "gliding": gl,
            "knobs": self.knobs, "ds": self.ds.state, "slice": self.ds.slice(),
            "out": [" ".join(f"{b:02X}" for b in m) for m in self.out_log], "log": self.log_lines,
            "glide_times": self.store.settings.get("glide_times"), "file": self.store.path,
        }

    def handle(self, cmd):
        k = cmd.get("type")
        if k == "key":
            self.engine.key(int(cmd["x"]), int(cmd["y"]), bool(cmd["down"]))
        elif k == "fader":
            self.move_fader(int(cmd["i"]), int(cmd["v"]))
        elif k == "knob":
            self.turn_knob(int(cmd["i"]), int(cmd["v"]))
        elif k == "ds":
            st = self.ds.state
            for f in ("block", "section", "fine"):
                if f in cmd:
                    st[f] = int(cmd[f])
            for f in ("left", "right"):
                if f in cmd:
                    st["freeze"][f] = bool(cmd[f])
        elif k == "tt":
            i = int(cmd["i"])
            self.tt_leds[i] = 0 if self.tt_leds[i] else 12
            self.tt.update(self.tt_leds)


PAGE = r"""<!DOCTYPE html><html><head><meta charset="utf-8"><title>Grid macros — simulator</title>
<style>
:root{--bg:#0c0f14;--panel:#13171d;--card:#1a1f27;--line:rgba(255,255,255,.09);--ink:#e0ebf7;--dim:rgba(255,255,255,.42);
--cv:#45dbf5;--trig:#ffa147;--special:#8cf08f;--fn:#fad65c}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:13px/1.45 -apple-system,system-ui,sans-serif}
header{padding:14px 22px;background:var(--panel);border-bottom:1px solid var(--line);display:flex;gap:16px;align-items:baseline}
header h1{font-size:16px;margin:0;font-weight:600}header span{color:var(--dim)}
main{display:grid;grid-template-columns:auto 1fr;gap:22px;padding:22px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px 16px}
.card h2{font-size:11px;letter-spacing:.08em;text-transform:uppercase;color:var(--dim);margin:0 0 10px;font-weight:600}
#grid{display:grid;grid-template-columns:repeat(8,34px) 14px repeat(8,34px);gap:6px;user-select:none}
.k{width:34px;height:34px;border-radius:6px;background:#22262d;border:1px solid rgba(0,0,0,.6);cursor:pointer}
.k.tt{cursor:pointer;opacity:.85}.gap{width:14px}
.labels{display:grid;grid-template-columns:repeat(8,34px);gap:6px;margin-top:6px;color:var(--dim);font-size:10px;text-align:center}
.legend{margin-top:12px;color:var(--dim);font-size:12px;max-width:640px}.legend b{color:var(--ink);font-weight:600}
.slots{display:grid;grid-template-columns:repeat(7,1fr);gap:6px;margin-top:12px}
.slot{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:7px;font-size:11px;min-height:74px}
.slot.on{border-color:var(--fn)}.slot b{display:block;font-size:12px}.slot .d{color:var(--dim)}
.rig{display:grid;gap:14px;align-content:start}
.faders{display:flex;gap:12px}.f{display:flex;flex-direction:column;align-items:center;gap:4px;font-size:10px;color:var(--dim)}
.f input{writing-mode:vertical-lr;direction:rtl;height:120px;width:22px;accent-color:var(--cv)}
.f .p{font-variant-numeric:tabular-nums;color:var(--ink)}.f.armed input{accent-color:var(--trig)}.f.armed .p{color:var(--trig)}
.f.gl .p{color:var(--special)}
.knobs{display:flex;gap:14px}.kn{text-align:center;font-size:10px;color:var(--dim)}.kn input{width:110px;accent-color:var(--cv)}
.row{display:flex;gap:16px;align-items:center;flex-wrap:wrap}.row label{color:var(--dim);font-size:12px}
.row input[type=range]{width:110px;accent-color:var(--fn)}
.mono{font:11px/1.5 ui-monospace,Menlo,monospace;color:var(--dim);white-space:pre}
.big{font-size:20px;font-variant-numeric:tabular-nums;color:var(--fn)}
.note{color:var(--dim);font-size:11px;margin-top:8px}
</style></head><body>
<header><h1>Grid macros</h1><span>simulator · the real engine with stand-ins for the rig</span><span id="file" style="margin-left:auto"></span></header>
<main>
<section>
 <div class="card"><h2>monome grid · left: macros (Pi) · right: Teletype</h2>
  <div id="grid"></div>
  <div class="labels"><span>bank</span><span>1</span><span>2</span><span>3</span><span>4</span><span>5</span><span>6</span><span>7</span></div>
  <div class="legend"><b>Column 1</b> banks · <b>top row</b> save (hold ½ s) · <b>rows 2–7</b> slew, bottom = instant ·
  <b>bottom row</b> load · <b>shift-click load</b> = clear (on the grid: hold save + press load).
  Right half: click to change what "Teletype" shows (kept with each snapshot, not restored yet).</div>
  <div class="slots" id="slots"></div>
 </div>
</section>
<section class="rig">
 <div class="card"><h2>16n · ch 16 · CC 80–87</h2><div class="faders" id="faders"></div>
  <div class="note">Number = the parameter's value. <span style="color:var(--trig)">Orange</span>: waiting for pickup — move the fader past the value.
  <span style="color:var(--special)">Green</span>: gliding.</div></div>
 <div class="card"><h2>EC4 · ch 1 · CC 1–4 (gets values sent back)</h2><div class="knobs" id="knobs"></div></div>
 <div class="card"><h2>DualSense script</h2>
  <div class="row"><label>block <input type="range" min="0" max="7" id="block"></label>
  <label>section <input type="range" min="0" max="7" id="section"></label>
  <label>fine <input type="range" min="0" max="15" id="fine"></label></div>
  <div class="row" style="margin-top:8px"><label><input type="checkbox" id="left"> freeze L2/R2/left stick</label>
  <label><input type="checkbox" id="right"> freeze right stick</label><span>slice <span class="big" id="slice"></span></span></div></div>
 <div class="card"><h2>Out through Grid_Macros · log</h2><div class="mono" id="out"></div><div class="mono" id="log" style="margin-top:8px;color:var(--ink)"></div></div>
</section></main>
<script>
const G=document.getElementById('grid');const cells=[];
function post(o){return fetch('/cmd',{method:'POST',body:JSON.stringify(o)})}
for(let y=0;y<8;y++){for(let x=0;x<17;x++){
 if(x==8){const g=document.createElement('div');g.className='gap';G.appendChild(g);continue}
 const d=document.createElement('div');const gx=x>8?x-1:x;d.className='k'+(gx>=8?' tt':'');
 if(gx<8){
  d.onmousedown=e=>{e.preventDefault();
   if(e.shiftKey&&y==7&&gx>0){post({type:'key',x:gx,y:0,down:true}).then(()=>post({type:'key',x:gx,y:7,down:true})).then(()=>post({type:'key',x:gx,y:0,down:false}));return}
   d.dataset.held=1;post({type:'key',x:gx,y,down:true})};
  const up=()=>{if(d.dataset.held){delete d.dataset.held;post({type:'key',x:gx,y,down:false})}};
  d.onmouseup=up;d.onmouseleave=up;
 } else d.onclick=()=>post({type:'tt',i:y*8+(gx-8)});
 G.appendChild(d);cells.push([d,gx,y]);
}}
function led(v,tt){if(!v)return '#22262d';const a=v/15;return tt?`rgba(140,200,255,${.15+.85*a})`:`rgba(255,${190+40*a|0},${90+60*a|0},${.12+.88*a})`}
const F=document.getElementById('faders');const fs=[];
for(let i=0;i<8;i++){const w=document.createElement('div');w.className='f';
 w.innerHTML=`<span class="p">–</span><input type="range" min="0" max="127"><span>${i+1}</span>`;
 const r=w.querySelector('input');r.oninput=()=>post({type:'fader',i,v:+r.value});F.appendChild(w);fs.push(w)}
const K=document.getElementById('knobs');const ks=[];
for(let i=0;i<4;i++){const w=document.createElement('div');w.className='kn';
 w.innerHTML=`<input type="range" min="0" max="127"><div>enc ${i+1} · <span>0</span></div>`;
 const r=w.querySelector('input');r.oninput=()=>post({type:'knob',i,v:+r.value});K.appendChild(w);ks.push(w)}
for(const id of ['block','section','fine']){const r=document.getElementById(id);r.oninput=()=>post({type:'ds',[id]:+r.value})}
for(const id of ['left','right']){const c=document.getElementById(id);c.onchange=()=>post({type:'ds',[id]:c.checked})}
let busy=false;
async function poll(){if(busy)return;busy=true;try{const s=await (await fetch('/state')).json();
 for(const [d,x,y] of cells){d.style.background=x<8?led(s.leds[y*8+x]):led(s.tt[y*8+x-8],true)}
 document.getElementById('file').textContent=s.file;
 const sl=document.getElementById('slots');sl.innerHTML='';
 s.slots.forEach((o,i)=>{const e=document.createElement('div');e.className='slot'+(o.active?' on':'');
  e.innerHTML=`<b>${s.bank+1}.${i+1}</b><span class="d">${o.saved?('saved '+o.when):'empty'} · ${o.glide}s</span>`+
  (o.parts||[]).map(p=>`<div class="d">${p}</div>`).join('');sl.appendChild(e)});
 s.faders.forEach((v,i)=>{const w=fs[i];const r=w.querySelector('input');if(document.activeElement!==r)r.value=v;
  w.querySelector('.p').textContent=s.params[i]??'–';w.classList.toggle('armed',s.armed[i]);w.classList.toggle('gl',s.gliding[i])});
 s.knobs.forEach((v,i)=>{const r=ks[i].querySelector('input');if(document.activeElement!==r)r.value=v;ks[i].querySelector('span').textContent=v});
 for(const id of ['block','section','fine']){const r=document.getElementById(id);if(document.activeElement!==r)r.value=s.ds[id]}
 document.getElementById('left').checked=s.ds.freeze.left;document.getElementById('right').checked=s.ds.freeze.right;
 document.getElementById('slice').textContent=s.slice;
 document.getElementById('out').textContent=s.out.join('\n')||'(nothing sent yet)';
 document.getElementById('log').textContent=s.log.join('\n');
}catch(e){}busy=false}
setInterval(poll,50);poll();
</script></body></html>"""


def make_handler(sim):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == "/":
                self._send(200, PAGE, "text/html; charset=utf-8")
            elif self.path == "/favicon.ico":
                self._send(204, b"", "image/x-icon")
            elif self.path == "/state":
                with sim.lock:
                    body = json.dumps(sim.state())
                self._send(200, body, "application/json")
            else:
                self._send(404, "not found", "text/plain")

        def do_POST(self):
            if self.path != "/cmd":
                self._send(404, "not found", "text/plain")
                return
            n = int(self.headers.get("Content-Length") or 0)
            try:
                cmd = json.loads(self.rfile.read(n) or b"{}")
                with sim.lock:
                    sim.handle(cmd)
                self._send(200, "{}", "application/json")
            except (ValueError, KeyError, TypeError) as e:
                self._send(400, json.dumps({"error": str(e)}), "application/json")
    return H


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8301)
    ap.add_argument("--store", default=os.path.join(live_config.DIR, "grid_macros_sim.json"))
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)
    sim = Sim(args.store)
    threading.Thread(target=sim.tick_forever, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(sim))
    url = f"http://localhost:{args.port}"
    print(f"Grid macros simulator: {url}   (snapshots in {args.store})", flush=True)
    if not args.no_browser:
        try:
            webbrowser.open(url)
        except Exception:
            pass
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
