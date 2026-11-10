#ifndef WEB_UI_H
#define WEB_UI_H

// =====================================================================
//  Embedded single-page control dashboard (served at GET /)
//  Gold (#DAA520) & dark-grey (#212121) theme, no external resources.
// =====================================================================

#include <Arduino.h>

const char WEB_UI_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="description" content="BrazoRobotica control dashboard: manual joint control, iterative inverse kinematics, emergency stop and live camera for the ESP32 robotic arm.">
<meta name="theme-color" content="#212121">
<meta name="color-scheme" content="dark">
<title>BrazoRobotica · Arm Control</title>
<style>
:root{
  --bg:#212121;--bg2:#1a1a1a;--card:#2a2a2a;--card2:#242424;--line:rgba(218,165,32,.16);
  --gold:#DAA520;--gold-l:#F3D27A;--gold-d:#9C7413;--txt:#ECE6D6;--mute:#9b968a;
  --ok:#8FC77E;--warn:#E8A33D;--bad:#E05A4F;--r:14px;
  --grad-gold:linear-gradient(135deg,var(--gold-l),var(--gold) 45%,var(--gold-d));
  --font:system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif;
  --mono:ui-monospace,"SFMono-Regular",Consolas,"Liberation Mono",monospace;
  color-scheme:dark;accent-color:var(--gold);
}
*{box-sizing:border-box}
html{scrollbar-color:var(--gold-d) var(--bg2)}
body{margin:0;font:15px/1.45 var(--font);color:var(--txt);min-height:100vh;
  background:radial-gradient(1200px 600px at 85% -10%,rgba(218,165,32,.10),transparent 60%),
             radial-gradient(900px 500px at -10% 110%,rgba(218,165,32,.06),transparent 60%),
             linear-gradient(180deg,#252525,var(--bg2));}
::selection{background:var(--gold);color:#1a1a1a}
h1,h2,h3{margin:0;font-weight:650;letter-spacing:.2px}
button,input,select{font:inherit;color:inherit}
.wrap{max-width:1280px;margin:0 auto;padding:18px}

/* ---------- header ---------- */
header{position:sticky;top:0;z-index:20;backdrop-filter:blur(10px);-webkit-backdrop-filter:blur(10px);
  background:linear-gradient(180deg,rgba(33,33,33,.92),rgba(33,33,33,.75));border-bottom:1px solid var(--line)}
.hbar{display:flex;align-items:center;gap:16px;flex-wrap:wrap;max-width:1280px;margin:0 auto;padding:12px 18px}
.brand{display:flex;align-items:center;gap:12px;margin-right:auto}
.logo{width:38px;height:38px;border-radius:11px;background:var(--grad-gold);display:grid;place-items:center;
  box-shadow:0 4px 18px rgba(218,165,32,.35),inset 0 1px 0 rgba(255,255,255,.4)}
.logo svg{width:24px;height:24px}
h1{font-size:19px;background:var(--grad-gold);-webkit-background-clip:text;background-clip:text;color:transparent}
.sub{font-size:12px;color:var(--mute)}
.pills{display:flex;gap:8px;flex-wrap:wrap}
.pill{display:inline-flex;align-items:center;gap:7px;padding:5px 11px;border-radius:99px;font-size:12px;
  background:rgba(255,255,255,.04);border:1px solid rgba(255,255,255,.08);color:var(--mute);transition:.3s}
.pill i{width:8px;height:8px;border-radius:50%;background:var(--mute);transition:.3s}
.pill.on{color:var(--txt)} .pill.on i{background:var(--ok);box-shadow:0 0 10px var(--ok)}
.pill.warn i{background:var(--warn);box-shadow:0 0 10px var(--warn)}
.pill.bad i{background:var(--bad);box-shadow:0 0 10px var(--bad)}

/* ---------- emergency stop ---------- */
#estop{position:relative;border:0;cursor:pointer;padding:11px 22px;border-radius:12px;font-weight:800;letter-spacing:1.2px;
  color:#fff;background:linear-gradient(180deg,#e2574c,#a5271e);min-width:170px;
  box-shadow:0 0 0 2px rgba(224,90,79,.35),0 6px 24px rgba(224,90,79,.35),inset 0 1px 0 rgba(255,255,255,.3);
  transition:transform .12s,box-shadow .2s}
#estop:hover{transform:translateY(-1px);box-shadow:0 0 0 3px rgba(224,90,79,.5),0 10px 30px rgba(224,90,79,.45)}
#estop:active{transform:translateY(1px) scale(.98)}
#estop small{display:block;font-size:10px;font-weight:600;letter-spacing:.5px;opacity:.85}
body.locked #estop{animation:pulse 1s infinite}
@keyframes pulse{50%{box-shadow:0 0 0 8px rgba(224,90,79,.15),0 6px 30px rgba(224,90,79,.6)}}
.lockbar{height:3px;background:var(--bad);transform-origin:left;transform:scaleX(0);transition:transform .1s linear}

/* ---------- layout ---------- */
.grid{display:grid;gap:18px;grid-template-columns:minmax(0,1.15fr) minmax(0,1fr)}
@media (max-width:960px){.grid{grid-template-columns:1fr}}
.col{display:flex;flex-direction:column;gap:18px;min-width:0}
.card{background:linear-gradient(180deg,var(--card),var(--card2));border:1px solid var(--line);border-radius:var(--r);
  padding:18px;box-shadow:0 10px 30px rgba(0,0,0,.25),inset 0 1px 0 rgba(255,255,255,.03);
  animation:rise .5s both}
.card:nth-child(2){animation-delay:.06s}.card:nth-child(3){animation-delay:.12s}
@keyframes rise{from{opacity:0;transform:translateY(8px)}}
.ch{display:flex;align-items:center;gap:10px;margin-bottom:14px}
.ch h2{font-size:15px;text-transform:uppercase;letter-spacing:1.4px;color:var(--gold-l)}
.ch .tag{margin-left:auto;font-size:12px;color:var(--mute)}
.ch::before{content:"";width:4px;height:18px;border-radius:2px;background:var(--grad-gold)}

/* ---------- controls ---------- */
.btn{border:1px solid rgba(218,165,32,.35);background:linear-gradient(180deg,#333,#2a2a2a);padding:9px 16px;border-radius:10px;
  cursor:pointer;transition:transform .12s,border-color .2s,background .2s,box-shadow .2s;white-space:nowrap}
.btn:hover:not(:disabled){border-color:var(--gold);box-shadow:0 0 0 3px rgba(218,165,32,.12)}
.btn:active:not(:disabled){transform:scale(.97)}
.btn.gold{background:var(--grad-gold);color:#1b1b1b;font-weight:700;border-color:transparent;box-shadow:0 4px 16px rgba(218,165,32,.25)}
.btn.gold:hover:not(:disabled){box-shadow:0 6px 22px rgba(218,165,32,.4)}
.btn.danger{border-color:rgba(224,90,79,.5)}
.btn:disabled{opacity:.4;cursor:not-allowed}
.btn:focus-visible,input:focus-visible,select:focus-visible,#estop:focus-visible{outline:2px solid var(--gold-l);outline-offset:2px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
.num,select,.txt{background:#1c1c1c;border:1px solid rgba(255,255,255,.1);border-radius:9px;padding:7px 9px;min-width:0;
  transition:border-color .2s}
.num:hover,select:hover,.txt:hover{border-color:rgba(218,165,32,.4)}
.num{width:84px;font-family:var(--mono);text-align:right}
input:not([type=range]):user-invalid{border-color:var(--bad)}
label.f{display:flex;flex-direction:column;gap:5px;font-size:12px;color:var(--mute)}

/* range slider (separate vendor rules on purpose) */
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:24px;background:transparent;cursor:pointer;
  --p:50%}
input[type=range]:disabled{cursor:not-allowed;opacity:.45}
input[type=range]::-webkit-slider-runnable-track{height:6px;border-radius:6px;
  background:linear-gradient(90deg,var(--gold-d),var(--gold) var(--p),#3a3a3a var(--p))}
input[type=range]::-moz-range-track{height:6px;border-radius:6px;background:#3a3a3a}
input[type=range]::-moz-range-progress{height:6px;border-radius:6px;background:linear-gradient(90deg,var(--gold-d),var(--gold))}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:18px;height:18px;margin-top:-6px;border-radius:50%;
  background:radial-gradient(circle at 35% 30%,#fff3c8,var(--gold) 55%,var(--gold-d));border:2px solid #1b1b1b;
  box-shadow:0 0 0 1px var(--gold),0 2px 8px rgba(0,0,0,.5);transition:transform .15s}
input[type=range]::-moz-range-thumb{width:16px;height:16px;border-radius:50%;
  background:radial-gradient(circle at 35% 30%,#fff3c8,var(--gold) 55%,var(--gold-d));border:2px solid #1b1b1b;
  box-shadow:0 0 0 1px var(--gold),0 2px 8px rgba(0,0,0,.5)}
input[type=range]:active::-webkit-slider-thumb{transform:scale(1.2)}

/* switch (styled checkbox, role=switch) */
.sw{-webkit-appearance:none;appearance:none;width:42px;height:24px;border-radius:24px;background:#3a3a3a;position:relative;
  cursor:pointer;transition:background .25s;border:1px solid rgba(255,255,255,.08);margin:0;flex:none}
.sw::after{content:"";position:absolute;top:2px;left:2px;width:18px;height:18px;border-radius:50%;background:#bdb7a8;
  transition:transform .25s,background .25s}
.sw:checked{background:var(--grad-gold)} .sw:checked::after{transform:translateX(18px);background:#1b1b1b}
.swl{display:flex;align-items:center;gap:10px;cursor:pointer;font-size:14px}

/* ---------- joints ---------- */
.joint{padding:10px 12px;border-radius:11px;background:rgba(0,0,0,.18);border:1px solid rgba(255,255,255,.04);transition:border-color .2s}
.joint+.joint{margin-top:8px}
.joint:hover{border-color:var(--line)}
.jh{display:flex;justify-content:space-between;font-size:13px;margin-bottom:2px}
.jn{font-weight:600}.jn b{color:var(--gold);font-family:var(--mono);margin-right:6px}
.ja{color:var(--mute);font-family:var(--mono);font-size:12px}
.jc{display:flex;gap:12px;align-items:center}
.jr{font-size:11px;color:var(--mute);font-family:var(--mono);display:flex;justify-content:space-between;margin-top:-2px;padding-right:96px}

/* ---------- IK ---------- */
.ikg{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
.ikg .num{width:100%}
.readout{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:14px}
.ro{background:rgba(0,0,0,.22);border:1px solid rgba(255,255,255,.05);border-radius:10px;padding:8px 10px}
.ro span{display:block;font-size:11px;color:var(--mute);text-transform:uppercase;letter-spacing:1px}
.ro b{font-family:var(--mono);font-size:17px;color:var(--gold-l)}
.ikres{margin-top:12px;font-size:13px;padding:10px 12px;border-radius:10px;background:rgba(0,0,0,.2);border-left:3px solid var(--mute);
  font-family:var(--mono);min-height:42px;transition:border-color .3s}
.ikres.ok{border-color:var(--ok)}.ikres.fail{border-color:var(--bad)}

/* ---------- camera ---------- */
.view{position:relative;aspect-ratio:4/3;border-radius:12px;overflow:hidden;background:
  repeating-linear-gradient(45deg,#1d1d1d 0 12px,#202020 12px 24px);border:1px solid var(--line);display:grid;place-items:center}
.view img{width:100%;height:100%;object-fit:contain;display:block}
.view .ph{color:var(--mute);font-size:13px;text-align:center;padding:20px}
.view .live{position:absolute;top:10px;left:10px;font-size:11px;font-weight:700;letter-spacing:1px;padding:3px 8px;border-radius:6px;
  background:rgba(224,90,79,.9);display:none}
.view.streaming .live{display:block}
.camg{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin-top:14px}
.camg .f input[type=range]{height:20px}
.shots{display:flex;gap:8px;margin-top:12px;overflow-x:auto}
.shots a{flex:none;width:96px;aspect-ratio:4/3;border-radius:8px;overflow:hidden;border:1px solid var(--line);transition:transform .15s}
.shots a:hover{transform:translateY(-2px);border-color:var(--gold)}
.shots img{width:100%;height:100%;object-fit:cover}

/* ---------- settings ---------- */
details.card>summary{cursor:pointer;list-style:none;display:flex;align-items:center;gap:10px}
details.card>summary::-webkit-details-marker{display:none}
details.card>summary::after{content:"\25BE";margin-left:auto;color:var(--gold);transition:transform .2s}
details[open].card>summary::after{transform:rotate(180deg)}
details.card>summary h2{font-size:15px;text-transform:uppercase;letter-spacing:1.4px;color:var(--gold-l)}
details.card>summary::before{content:"";width:4px;height:18px;border-radius:2px;background:var(--grad-gold)}
.tbl{width:100%;overflow-x:auto;margin-top:14px}
table{border-collapse:collapse;width:100%;font-size:13px}
th{font-size:11px;font-weight:600;color:var(--mute);text-transform:uppercase;letter-spacing:.8px;text-align:left;padding:6px}
td{padding:4px 6px;border-top:1px solid rgba(255,255,255,.05)}
td .num{width:72px}
.sec{margin-top:18px;padding-top:16px;border-top:1px solid var(--line)}
.sec h3{font-size:13px;color:var(--gold-l);margin-bottom:10px;text-transform:uppercase;letter-spacing:1px}
.hint{font-size:12px;color:var(--mute);margin-top:8px}
code{font-family:var(--mono);color:var(--gold-l)}

/* ---------- footer / toast ---------- */
footer{color:var(--mute);font-size:12px;text-align:center;padding:24px 0 30px;font-family:var(--mono)}
#toasts{position:fixed;right:16px;bottom:16px;display:flex;flex-direction:column;gap:8px;z-index:50;max-width:min(380px,calc(100vw - 32px))}
.toast{padding:11px 14px;border-radius:10px;background:linear-gradient(180deg,#333,#2a2a2a);border:1px solid var(--line);
  border-left:3px solid var(--gold);box-shadow:0 10px 30px rgba(0,0,0,.4);font-size:13px;animation:tin .25s both}
.toast.err{border-left-color:var(--bad)}
@keyframes tin{from{opacity:0;transform:translateX(20px)}}
body.locked .mvz{position:relative}
body.locked .mvz::after{content:"MOVEMENT LOCKED";position:absolute;inset:0;display:grid;place-items:center;border-radius:var(--r);
  background:rgba(20,20,20,.55);color:var(--bad);font-weight:800;letter-spacing:2px;font-size:13px;pointer-events:none}
@media (prefers-reduced-motion:reduce){*{animation:none!important;transition:none!important}}
</style>
</head>
<body>
<header>
  <div class="hbar">
    <div class="brand">
      <div class="logo" aria-hidden="true"><svg viewBox="0 0 24 24" fill="none" stroke="#1b1b1b" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M4 21h9"/><path d="M8 21v-5l4-6"/><path d="M12 10l6-3"/><circle cx="12" cy="10" r="1.6"/><path d="M18 7l2 2M18 7l1-3"/></svg></div>
      <div><h1>BrazoRobotica</h1><div class="sub" id="host">connecting…</div></div>
    </div>
    <div class="pills" role="status" aria-live="polite">
      <span class="pill" id="pConn"><i></i>Link</span>
      <span class="pill" id="pPwm"><i></i>Servos</span>
      <span class="pill" id="pCam"><i></i>Camera</span>
    </div>
    <button id="estop" type="button" aria-keyshortcuts="Escape" title="Emergency stop (Esc)">EMERGENCY STOP<small id="estopTxt">press or hit Esc</small></button>
  </div>
  <div class="lockbar" id="lockbar"></div>
</header>

<main class="wrap">
  <div class="grid">
    <div class="col">
      <section class="card mvz" aria-labelledby="hJ">
        <div class="ch"><h2 id="hJ">Manual Control</h2><span class="tag" id="moving">idle</span></div>
        <div id="joints"></div>
        <div class="sec row">
          <button class="btn gold mv" id="bHome" type="button">Home position</button>
          <button class="btn" id="bSync" type="button">Sync sliders to arm</button>
          <label class="f" style="flex:1;min-width:180px">Max speed <span id="spdV" style="color:var(--txt);font-family:var(--mono)"></span>
            <input type="range" id="spd" min="5" max="360" step="5"></label>
        </div>
      </section>

      <section class="card mvz" aria-labelledby="hIK">
        <div class="ch"><h2 id="hIK">Inverse Kinematics</h2><span class="tag">iterative · damped least squares</span></div>
        <div class="ikg">
          <label class="f">X (mm)<input class="num" id="ikX" type="number" step="1" value="150" required></label>
          <label class="f">Y (mm)<input class="num" id="ikY" type="number" step="1" value="0" required></label>
          <label class="f">Z (mm)<input class="num" id="ikZ" type="number" step="1" value="60" required></label>
          <label class="f">Pitch (°)<input class="num" id="ikP" type="number" step="1" value="-90" min="-180" max="180"></label>
        </div>
        <div class="row" style="margin-top:12px">
          <label class="swl"><input class="sw" id="ikO" type="checkbox" role="switch">Constrain tool pitch</label>
          <span style="margin-left:auto"></span>
          <button class="btn" id="bPrev" type="button">Preview</button>
          <button class="btn gold mv" id="bIK" type="button">Solve &amp; move</button>
        </div>
        <div class="readout" aria-label="Tool tip position (forward kinematics)">
          <div class="ro"><span>Tip X</span><b id="tX">—</b></div>
          <div class="ro"><span>Tip Y</span><b id="tY">—</b></div>
          <div class="ro"><span>Tip Z</span><b id="tZ">—</b></div>
        </div>
        <div class="ikres" id="ikRes" aria-live="polite">Enter a target and press <b>Solve &amp; move</b>.</div>
        <div class="row" style="margin-top:10px"><button class="btn" id="bUseTip" type="button">Use current tip as target</button></div>
      </section>
    </div>

    <div class="col">
      <section class="card" aria-labelledby="hC">
        <div class="ch"><h2 id="hC">Vision</h2><span class="tag" id="camTag">—</span></div>
        <div class="view" id="view"><span class="live">LIVE</span><div class="ph" id="ph">Stream stopped</div><img id="img" alt="Camera stream" hidden></div>
        <div class="row" style="margin-top:12px">
          <button class="btn gold" id="bStream" type="button">Start stream</button>
          <button class="btn" id="bShot" type="button">Take photo</button>
          <label class="swl" style="margin-left:auto"><input class="sw" id="cLed" type="checkbox" role="switch">Flash LED</label>
        </div>
        <div class="camg">
          <label class="f">Resolution<select id="cRes"></select></label>
          <label class="f">JPEG quality <span id="cQv"></span><input type="range" id="cQ" min="4" max="63" step="1"></label>
          <label class="f">Brightness <span id="cBv"></span><input type="range" id="cB" min="-2" max="2" step="1"></label>
          <label class="f">Contrast <span id="cCv"></span><input type="range" id="cC" min="-2" max="2" step="1"></label>
          <label class="f">Saturation <span id="cSv"></span><input type="range" id="cS" min="-2" max="2" step="1"></label>
          <div class="f" style="justify-content:flex-end;gap:8px">
            <label class="swl"><input class="sw" id="cH" type="checkbox" role="switch">Mirror</label>
            <label class="swl"><input class="sw" id="cV" type="checkbox" role="switch">Flip</label>
          </div>
        </div>
        <div class="shots" id="shots" aria-label="Captured photos"></div>
      </section>

      <details class="card" id="settings">
        <summary><h2>Settings &amp; Calibration</h2></summary>
        <div class="sec">
          <h3>Network address</h3>
          <div class="row">
            <input class="txt" id="sHost" type="text" pattern="[a-z0-9]([a-z0-9.\-]*[a-z0-9])?" maxlength="63" spellcheck="false" autocomplete="off" aria-label="Hostname" style="flex:1;min-width:180px">
            <button class="btn gold" id="bHost" type="button">Save hostname</button>
          </div>
          <p class="hint">UI address: <code id="sUrl"></code> — takes effect after restart. Fallback: <code>http://192.168.4.1/</code></p>
        </div>
        <div class="sec">
          <h3>Joint calibration</h3>
          <div class="tbl"><table><thead><tr><th>Joint</th><th>Ch</th><th>Min µs</th><th>Max µs</th><th>Min °</th><th>Max °</th><th>Home °</th><th>Kin 0 °</th><th>Dir</th></tr></thead><tbody id="calB"></tbody></table></div>
          <h3 style="margin-top:16px">Link lengths (mm)</h3>
          <div class="ikg">
            <label class="f">L0 base<input class="num" id="L0" type="number" step="0.5" min="0"></label>
            <label class="f">L1 upper arm<input class="num" id="L1" type="number" step="0.5" min="1"></label>
            <label class="f">L2 forearm<input class="num" id="L2" type="number" step="0.5" min="1"></label>
            <label class="f">L3 hand+tool<input class="num" id="L3" type="number" step="0.5" min="0"></label>
          </div>
          <div class="row" style="margin-top:14px"><button class="btn gold" id="bCal" type="button">Save calibration</button></div>
          <p class="hint">"Kin 0" is the servo angle where the joint is at its kinematic zero (links straight up / aligned). Values are stored in the controller's non-volatile memory.</p>
        </div>
        <div class="sec row">
          <button class="btn" id="bRestart" type="button">Restart controller</button>
          <button class="btn danger" id="bReset" type="button">Factory reset</button>
        </div>
      </details>
    </div>
  </div>
  <footer id="foot">BrazoRobotica · ESP32-CAM</footer>
</main>
<div id="toasts" aria-live="polite"></div>

<script>
"use strict";
const $=s=>document.querySelector(s);
const FS=[[5,"QVGA 320×240"],[6,"CIF 400×296"],[8,"VGA 640×480"],[9,"SVGA 800×600"],[10,"XGA 1024×768"],[11,"HD 1280×720"],[12,"SXGA 1280×1024"],[13,"UXGA 1600×1200"]];
const S={ws:null,cfg:null,st:null,lockUntil:0,lockMs:3000,retry:500,pend:{},timer:0,streaming:false};
const f1=v=>(+v).toFixed(1);
const clamp=(v,a,b)=>Math.min(b,Math.max(a,v));

function toast(msg,kind){const t=document.createElement("div");t.className="toast"+(kind==="err"?" err":"");t.textContent=msg;
  $("#toasts").append(t);setTimeout(()=>t.remove(),kind==="err"?5000:3000);}

/* ---------------- WebSocket ---------------- */
function connect(){
  const ws=new WebSocket(`ws://${location.host}/ws`);S.ws=ws;
  ws.onopen=()=>{S.retry=500;pill("#pConn","on");};
  ws.onclose=()=>{pill("#pConn","bad");setTimeout(connect,S.retry);S.retry=Math.min(S.retry*2,5000);};
  ws.onerror=()=>ws.close();
  ws.onmessage=e=>{let m;try{m=JSON.parse(e.data)}catch{return}(H[m.type]||(()=>{}))(m);};
}
function send(o){if(S.ws&&S.ws.readyState===1){S.ws.send(JSON.stringify(o));return true}toast("Not connected to the arm","err");return false}
const locked=()=>Date.now()<S.lockUntil;
function sendMove(o){if(locked()){toast("Emergency stop active – movement locked","err");return false}return send(o)}

/* ---------------- message handlers ---------------- */
const H={
  config(m){const first=!S.cfg;S.cfg=m;S.lockMs=m.lockoutMs||3000;
    $("#host").textContent=m.url;$("#sUrl").textContent=m.url;
    if(document.activeElement!==$("#sHost"))$("#sHost").value=m.hostname;
    $("#spd").value=m.maxSpeed;$("#spdV").textContent=m.maxSpeed+" °/s";fill($("#spd"));
    buildJoints();buildCal();fillCamera(m.camera,m.psram);
    ["L0","L1","L2","L3"].forEach(k=>$("#"+k).value=m.model[k]);
    if(first&&S.st)sync();},
  status(m){S.st=m;
    pill("#pPwm",m.pwm?"on":"bad");pill("#pCam",m.camera?"on":"warn");
    $("#moving").textContent=m.moving?"moving…":"idle";
    m.angles.forEach((a,i)=>{const e=$("#ja"+i);if(e)e.textContent="actual "+f1(a)+"°";});
    m.targets.forEach((a,i)=>setSlider(i,a,false));
    $("#tX").textContent=f1(m.tip[0]);$("#tY").textContent=f1(m.tip[1]);$("#tZ").textContent=f1(m.tip[2]);
    if(m.estop)S.lockUntil=Math.max(S.lockUntil,Date.now()+m.estop_ms);
    else if(S.lockUntil>Date.now()+400)S.lockUntil=Date.now();   // server released earlier
    $("#foot").textContent=`uptime ${(m.uptime_ms/1000|0)} s · heap ${(m.heap/1024|0)} KB · clients ${m.clients}`;},
  ik_result(m){const r=$("#ikRes");r.className="ikres "+(m.success?"ok":"fail");
    r.innerHTML=`${m.success?"✓ Converged":"✗ No exact solution"} · err ${m.posErr} mm${$("#ikO").checked?" / "+m.oriErr:""} · ${m.iterations} it · ${(m.time_us/1000).toFixed(1)} ms${m.moved?" · <b style='color:var(--ok)'>moving</b>":""}<br>θ = [${m.angles.join(", ")}]`;},
  info(m){toast(m.msg)},
  error(m){toast(m.msg,"err");if(m.remaining_ms)S.lockUntil=Date.now()+m.remaining_ms;}
};
function pill(sel,state){const p=$(sel);p.classList.remove("on","warn","bad");if(state)p.classList.add(state);}

/* ---------------- sliders ---------------- */
function fill(r){const a=+r.min,b=+r.max;r.style.setProperty("--p",((r.value-a)/(b-a)*100)+"%");}
function setSlider(i,v,force){const r=$("#js"+i),n=$("#jv"+i);if(!r)return;
  const busy=document.activeElement===r||document.activeElement===n||S.pend[i]!==undefined;
  if(busy&&!force)return;r.value=v;n.value=f1(v);fill(r);}
function queueJoint(i,v){S.pend[i]=v;if(S.timer)return;S.timer=setTimeout(flush,60);}
function flush(){S.timer=0;const p=S.pend;S.pend={};for(const k in p)sendMove({type:"set_joint",id:+k,angle:p[k]});}
function buildJoints(){const box=$("#joints");box.textContent="";
  S.cfg.joints.forEach((j,i)=>{const d=document.createElement("div");d.className="joint";
    d.innerHTML=`<div class="jh"><span class="jn"><b>J${i}</b>${j.name}</span><span class="ja" id="ja${i}">—</span></div>
    <div class="jc"><input class="mv" type="range" id="js${i}" min="${j.min}" max="${j.max}" step="0.5" aria-label="${j.name} angle">
    <input class="num mv" type="number" id="jv${i}" min="${j.min}" max="${j.max}" step="0.5" aria-label="${j.name} angle (degrees)"></div>
    <div class="jr"><span>${j.min}°</span><span>home ${j.home}°</span><span>${j.max}°</span></div>`;
    box.append(d);
    const r=d.querySelector("input[type=range]"),n=d.querySelector("input[type=number]");
    r.value=j.home;n.value=j.home;fill(r);
    r.addEventListener("input",()=>{n.value=f1(r.value);fill(r);queueJoint(i,+r.value);});
    n.addEventListener("change",()=>{const v=clamp(+n.value||0,j.min,j.max);n.value=f1(v);r.value=v;fill(r);sendMove({type:"set_joint",id:i,angle:v});});
  });
  if(S.st)S.st.targets.forEach((a,i)=>setSlider(i,a,true));
  applyLock(true);}
function sync(){if(S.st)S.st.angles.forEach((a,i)=>setSlider(i,a,true));}

/* ---------------- IK ---------------- */
function ik(move){const x=+$("#ikX").value,y=+$("#ikY").value,z=+$("#ikZ").value,p=+$("#ikP").value;
  if([x,y,z,p].some(Number.isNaN)){toast("Enter numeric X, Y, Z","err");return}
  const o={type:"ik_target",x,y,z,orient:$("#ikO").checked,pitch:p,move};
  if(move?sendMove(o):send(o))$("#ikRes").textContent="Solving…";}

/* ---------------- camera ---------------- */
function fillCamera(c,psram){const sel=$("#cRes");
  if(!sel.options.length)FS.forEach(([v,t])=>sel.add(new Option(t,v)));
  [...sel.options].forEach(o=>o.disabled=!psram&&+o.value>9);
  sel.value=c.framesize;
  [["#cQ",c.quality],["#cB",c.brightness],["#cC",c.contrast],["#cS",c.saturation]].forEach(([s,v])=>{$(s).value=v;$(s+"v").textContent=v;fill($(s));});
  $("#cH").checked=c.hmirror;$("#cV").checked=c.vflip;$("#cLed").checked=c.led;
  $("#camTag").textContent=c.ready?(psram?"OV2640 · PSRAM":"OV2640 · no PSRAM"):"camera offline";}
function camSend(k,v){send({type:"camera",[k]:v});}
function toggleStream(){const img=$("#img"),v=$("#view");
  if(S.streaming){img.removeAttribute("src");img.hidden=true;$("#ph").hidden=false;$("#ph").textContent="Stream stopped";
    v.classList.remove("streaming");S.streaming=false;$("#bStream").textContent="Start stream";return}
  const port=(S.cfg&&S.cfg.streamPort)||81;
  img.onerror=()=>{if(S.streaming){toast("Stream unavailable (another viewer may be connected)","err");toggleStream();}};
  img.src=`http://${location.hostname}:${port}/stream?t=${Date.now()}`;img.hidden=false;$("#ph").hidden=true;
  v.classList.add("streaming");S.streaming=true;$("#bStream").textContent="Stop stream";}
async function shot(){const b=$("#bShot");b.disabled=true;
  try{const r=await fetch("/capture?t="+Date.now(),{cache:"no-store"});if(!r.ok)throw new Error(await r.text());
    const url=URL.createObjectURL(await r.blob());const a=document.createElement("a");
    const ts=new Date().toISOString().replace(/[:.]/g,"-").slice(0,19);
    a.href=url;a.download=`brazo_${ts}.jpg`;a.title="Download "+a.download;a.innerHTML=`<img src="${url}" alt="Capture ${ts}">`;
    const box=$("#shots");box.prepend(a);while(box.children.length>6){URL.revokeObjectURL(box.lastChild.href);box.lastChild.remove();}
    if(!S.streaming){$("#img").src=url;$("#img").hidden=false;$("#ph").hidden=true;}
    toast("Photo captured – click the thumbnail to download");
  }catch(e){toast("Capture failed: "+e.message,"err")}finally{b.disabled=false}}

/* ---------------- settings ---------------- */
const CF=[["channel",0,15,1],["minUs",300,2800,10],["maxUs",300,2800,10],["min",0,360,0.5],["max",0,360,0.5],["home",0,360,0.5],["kinZero",0,360,0.5]];
function buildCal(){const tb=$("#calB");if(tb.contains(document.activeElement))return;tb.textContent="";
  S.cfg.joints.forEach((j,i)=>{const tr=document.createElement("tr");
    tr.innerHTML=`<td>${j.name}</td>`+CF.map(([k,a,b,s])=>`<td><input class="num" data-j="${i}" data-k="${k}" type="number" min="${a}" max="${b}" step="${s}" value="${j[k]}" aria-label="${j.name} ${k}"></td>`).join("")+
      `<td><select data-j="${i}" data-k="kinDir" aria-label="${j.name} direction"><option value="1">+1</option><option value="-1">−1</option></select></td>`;
    tb.append(tr);tr.querySelector("select").value=j.kinDir;});}
function saveCal(){const joints=S.cfg.joints.map(()=>({}));let bad=false;
  $("#calB").querySelectorAll("[data-k]").forEach(e=>{if(e.checkValidity&&!e.checkValidity())bad=true;joints[e.dataset.j][e.dataset.k]=+e.value;});
  if(bad){toast("Some calibration values are out of range","err");return}
  const model={};["L0","L1","L2","L3"].forEach(k=>model[k]=+$("#"+k).value);
  send({type:"save_calibration",joints,model});}

/* ---------------- emergency stop / lock ---------------- */
function estop(){S.lockUntil=Date.now()+S.lockMs;S.pend={};clearTimeout(S.timer);S.timer=0;
  send({type:"emergency_stop"});applyLock(true);toast("EMERGENCY STOP – movement locked for "+(S.lockMs/1000)+" s","err");}
let wasLocked=false;
function applyLock(force){const l=locked();
  if(l||force||l!==wasLocked){document.body.classList.toggle("locked",l);
    document.querySelectorAll(".mv").forEach(e=>e.disabled=l);
    const rem=Math.max(0,S.lockUntil-Date.now());
    $("#estopTxt").textContent=l?`locked ${(rem/1000).toFixed(1)} s`:"press or hit Esc";
    $("#lockbar").style.transform=`scaleX(${l?rem/S.lockMs:0})`;
    if(wasLocked&&!l)toast("Movement enabled");wasLocked=l;}}
setInterval(()=>applyLock(false),100);

/* ---------------- wiring ---------------- */
$("#estop").addEventListener("click",estop);
document.addEventListener("keydown",e=>{if(e.key==="Escape"){e.preventDefault();estop();}});
$("#bHome").addEventListener("click",()=>{if(sendMove({type:"home"}))S.cfg.joints.forEach((j,i)=>setSlider(i,j.home,true));});
$("#bSync").addEventListener("click",sync);
$("#spd").addEventListener("input",e=>{$("#spdV").textContent=e.target.value+" °/s";fill(e.target);});
$("#spd").addEventListener("change",e=>send({type:"set_speed",value:+e.target.value}));
$("#bIK").addEventListener("click",()=>ik(true));
$("#bPrev").addEventListener("click",()=>ik(false));
$("#bUseTip").addEventListener("click",()=>{if(!S.st)return;["#ikX","#ikY","#ikZ"].forEach((s,k)=>$(s).value=Math.round(S.st.tip[k]));});
$("#bStream").addEventListener("click",toggleStream);
$("#bShot").addEventListener("click",shot);
$("#cLed").addEventListener("change",e=>camSend("led",e.target.checked));
$("#cH").addEventListener("change",e=>camSend("hmirror",e.target.checked));
$("#cV").addEventListener("change",e=>camSend("vflip",e.target.checked));
$("#cRes").addEventListener("change",e=>camSend("framesize",+e.target.value));
[["#cQ","quality"],["#cB","brightness"],["#cC","contrast"],["#cS","saturation"]].forEach(([s,k])=>{
  $(s).addEventListener("input",e=>{$(s+"v").textContent=e.target.value;fill(e.target);});
  $(s).addEventListener("change",e=>camSend(k,+e.target.value));});
$("#bHost").addEventListener("click",()=>{const h=$("#sHost");h.value=h.value.trim().toLowerCase();
  if(!h.checkValidity()||!h.value){toast("Invalid hostname","err");return}send({type:"set_hostname",hostname:h.value});});
$("#bCal").addEventListener("click",saveCal);
$("#bRestart").addEventListener("click",()=>{if(confirm("Restart the controller?"))send({type:"restart"});});
$("#bReset").addEventListener("click",()=>{if(confirm("Restore factory defaults (hostname, calibration, camera)?"))send({type:"factory_reset"});});
connect();
</script>
</body>
</html>)rawliteral";

#endif // WEB_UI_H
