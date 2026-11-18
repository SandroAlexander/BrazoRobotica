#ifndef WEB_UI_H
#define WEB_UI_H

// =====================================================================
//  Embedded multi-page control dashboard (served at /, /queries, /ik, /settings)
//  Gold (#DAA520) & dark-grey (#212121) theme, no external resources.
// =====================================================================

#include <Arduino.h>

const char WEB_UI_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="description" content="BrazoRobotica control dashboard: manual joint control, movement queries, inverse kinematics, emergency stop and live camera for the ESP32 robotic arm.">
<meta name="theme-color" content="#212121">
<meta name="color-scheme" content="dark">
<link rel="icon" href="data:,">
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
[hidden]{display:none!important}
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
.hbar{display:flex;align-items:center;gap:14px;flex-wrap:wrap;max-width:1280px;margin:0 auto;padding:12px 18px}
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
.grid.eq{grid-template-columns:minmax(0,1fr) minmax(0,1fr)}
@media (max-width:960px){.grid,.grid.eq{grid-template-columns:1fr}}
.col{display:flex;flex-direction:column;gap:18px;min-width:0}
.card{background:linear-gradient(180deg,var(--card),var(--card2));border:1px solid var(--line);border-radius:var(--r);
  padding:18px;box-shadow:0 10px 30px rgba(0,0,0,.25),inset 0 1px 0 rgba(255,255,255,.03);
  animation:rise .35s both}
.card:nth-child(2){animation-delay:.05s}
@keyframes rise{from{opacity:0;transform:translateY(6px)}}
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
.btn.sm{padding:6px 10px;font-size:12px;border-radius:8px}
.btn:disabled{opacity:.4;cursor:not-allowed}
.btn:focus-visible,input:focus-visible,select:focus-visible,#estop:focus-visible{outline:2px solid var(--gold-l);outline-offset:2px}
.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}
.num,select,.txt{background:#1c1c1c;border:1px solid rgba(255,255,255,.1);border-radius:9px;padding:7px 9px;min-width:0;
  transition:border-color .2s}
.num:hover,select:hover,.txt:hover{border-color:rgba(218,165,32,.4)}
.num{width:84px;font-family:var(--mono);text-align:right}
.num:disabled{opacity:.45;cursor:not-allowed}
input:not([type=range]):user-invalid{border-color:var(--bad)}
label.f{display:flex;flex-direction:column;gap:5px;font-size:12px;color:var(--mute)}

/* range slider */
input[type=range]{-webkit-appearance:none;appearance:none;width:100%;height:24px;background:transparent;cursor:pointer;--p:50%}
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
input[type=range]:active:not(:disabled)::-webkit-slider-thumb{transform:scale(1.2)}

/* switch */
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
.spdh{display:flex;align-items:center;justify-content:space-between;gap:8px;width:100%}
.spdbox{flex:1;min-width:210px;padding:8px 10px;border-radius:10px;background:rgba(0,0,0,.18);border:1px solid rgba(255,255,255,.06)}
.spdbox.unlocked{border-color:rgba(218,165,32,.45);background:rgba(218,165,32,.05)}

/* ---------- queries window ---------- */
.qgrid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}
@media (max-width:560px){.qgrid{grid-template-columns:1fr}}
.qcell{padding:11px 12px;border-radius:11px;background:rgba(0,0,0,.20);border:1px solid rgba(255,255,255,.06);
  display:flex;flex-direction:column;gap:8px;transition:border-color .2s,box-shadow .2s,opacity .2s}
.qcell:hover{border-color:var(--line)}
.qcell.wild{border-color:rgba(255,255,255,.05);background:rgba(0,0,0,.12)}
.qcell.wild .qang{opacity:.4}
.qcell.bad{border-color:var(--bad)}
.qcell.seq-on{border-color:var(--gold);box-shadow:0 0 0 2px rgba(218,165,32,.25)}
.qch{display:flex;justify-content:space-between;align-items:center;gap:6px;font-size:13px}
.qch b{color:var(--gold);font-family:var(--mono);margin-right:5px}
.qcr{display:flex;align-items:center;gap:6px}
.qcr .num{flex:1;min-width:68px;width:auto}
.qslbl{font-size:11px;color:var(--mute);min-width:68px}
.qtoggle{border:1px solid rgba(255,255,255,.12);background:#252525;color:var(--mute);padding:6px 9px;border-radius:8px;
  font-size:12px;cursor:pointer;white-space:nowrap;transition:.15s}
.qtoggle:hover{border-color:var(--gold);color:var(--txt)}
.qtoggle.active{background:var(--grad-gold);color:#1b1b1b;font-weight:700;border-color:transparent}
.qacts{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:10px;margin-top:10px}
@media (max-width:600px){.qacts{grid-template-columns:1fr}}
.btn.estop-inline{border:0;font-weight:800;letter-spacing:.8px;color:#fff;background:linear-gradient(180deg,#e2574c,#a5271e);
  box-shadow:0 0 0 1px rgba(224,90,79,.4),0 4px 16px rgba(224,90,79,.3)}
.btn.estop-inline:hover:not(:disabled){box-shadow:0 0 0 2px rgba(224,90,79,.6),0 6px 22px rgba(224,90,79,.45)}

.qlist{display:flex;flex-direction:column;gap:10px;max-height:700px;overflow-y:auto;padding-right:2px}
.qitem{padding:11px 12px;border-radius:11px;background:rgba(0,0,0,.18);border:1px solid rgba(255,255,255,.05);transition:border-color .2s}
.qitem:hover{border-color:var(--line)}
.qitem-top{display:flex;align-items:center;justify-content:space-between;gap:8px;flex-wrap:wrap;margin-bottom:8px}
.qbtns{display:flex;gap:6px;align-items:center;flex-wrap:wrap}
.qtbl-wrap{overflow-x:auto}
.qtbl{width:100%;border-collapse:collapse;font-family:var(--mono);font-size:12px;text-align:center;
  background:rgba(0,0,0,.22);border-radius:8px;overflow:hidden}
.qtbl th{padding:4px 5px;font-size:10.5px;color:var(--mute);background:rgba(255,255,255,.03);border-bottom:1px solid rgba(255,255,255,.06);text-align:center}
.qtbl td{padding:4px 5px;border-top:0;border-right:1px solid rgba(255,255,255,.04)}
.qtbl td:last-child{border-right:0}
.qtbl .qrow-a td{color:var(--gold-l);font-weight:650}
.qtbl .qrow-a td.wild{color:var(--mute);font-weight:400}
.qtbl .qrow-s td{color:var(--mute);font-size:11px;border-top:1px solid rgba(255,255,255,.04)}
.qtbl .qrow-s td.spd-c{color:var(--txt)}
.qempty{padding:26px 16px;text-align:center;color:var(--mute);font-size:13px;border:1px dashed rgba(218,165,32,.18);border-radius:11px}
.membar{height:6px;border-radius:6px;background:#1a1a1a;overflow:hidden;margin-top:8px;border:1px solid rgba(255,255,255,.05)}
.membar>i{display:block;height:100%;width:0%;background:var(--grad-gold);transition:width .3s}

/* ---------- IK ---------- */
.ikg{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px}
.ikg .num{width:100%}
@media (max-width:540px){.ikg{grid-template-columns:repeat(2,minmax(0,1fr))}}
.readout{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;margin-top:14px}
.ro{background:rgba(0,0,0,.22);border:1px solid rgba(255,255,255,.05);border-radius:10px;padding:8px 10px}
.ro span{display:block;font-size:11px;color:var(--mute);text-transform:uppercase;letter-spacing:1px}
.ro b{font-family:var(--mono);font-size:17px;color:var(--gold-l)}
.ikres{margin-top:12px;font-size:13px;padding:10px 12px;border-radius:10px;background:rgba(0,0,0,.2);border-left:3px solid var(--mute);
  font-family:var(--mono);min-height:42px;transition:border-color .3s}
.ikres.ok{border-color:var(--ok)}.ikres.fail{border-color:var(--bad)}

/* ---------- camera & advanced filters ---------- */
.view{position:relative;aspect-ratio:4/3;border-radius:12px;overflow:hidden;background:
  repeating-linear-gradient(45deg,#1d1d1d 0 12px,#202020 12px 24px);border:1px solid var(--line);display:grid;place-items:center}
.view img{width:100%;height:100%;object-fit:contain;display:block;transition:filter .15s}
.view .ph{color:var(--mute);font-size:13px;text-align:center;padding:20px}
.view .live{position:absolute;top:10px;left:10px;font-size:11px;font-weight:700;letter-spacing:1px;padding:3px 8px;border-radius:6px;
  background:rgba(224,90,79,.9);display:none;z-index:2}
.view.streaming .live{display:block}
.cam-ov{position:absolute;inset:0;width:100%;height:100%;pointer-events:none;z-index:1}
.camg{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px;margin-top:14px}
.camg .f input[type=range]{height:20px}
.shots{display:flex;gap:8px;margin-top:12px;overflow-x:auto}
.shots a{flex:none;width:96px;aspect-ratio:4/3;border-radius:8px;overflow:hidden;border:1px solid var(--line);transition:transform .15s}
.shots a:hover{transform:translateY(-2px);border-color:var(--gold)}
.shots img{width:100%;height:100%;object-fit:cover}
details.subcard{margin-top:14px;padding:10px 12px;border-radius:11px;background:rgba(0,0,0,.18);border:1px solid var(--line)}
details.subcard>summary{cursor:pointer;list-style:none;display:flex;align-items:center;gap:8px;font-size:13px;font-weight:650;
  color:var(--gold-l);text-transform:uppercase;letter-spacing:.9px}
details.subcard>summary::-webkit-details-marker{display:none}
details.subcard>summary::after{content:"\25BE";margin-left:auto;color:var(--gold);transition:transform .2s}
details[open].subcard>summary::after{transform:rotate(180deg)}
.fx-pills{display:flex;gap:6px;flex-wrap:wrap;margin-top:10px}
.fxp{border:1px solid rgba(255,255,255,.1);background:#252525;color:var(--mute);padding:5px 10px;border-radius:99px;font-size:12px;cursor:pointer;transition:.15s}
.fxp:hover{border-color:var(--gold);color:var(--txt)}
.fxp.active{background:var(--grad-gold);color:#1b1b1b;font-weight:700;border-color:transparent}

/* ---------- settings ---------- */
.tbl{width:100%;overflow-x:auto;margin-top:8px}
table.calt{border-collapse:collapse;width:100%;font-size:13px}
table.calt th{font-size:11px;font-weight:600;color:var(--mute);text-transform:uppercase;letter-spacing:.8px;text-align:left;padding:6px}
table.calt td{padding:4px 6px;border-top:1px solid rgba(255,255,255,.05)}
table.calt td .num{width:72px}
.sec{margin-top:18px;padding-top:16px;border-top:1px solid var(--line)}
.sec h3{font-size:13px;color:var(--gold-l);margin-bottom:10px;text-transform:uppercase;letter-spacing:1px}
.hint{font-size:12px;color:var(--mute);margin-top:8px}
code{font-family:var(--mono);color:var(--gold-l)}

/* ---------- floating bottom-left menu (FAB) ---------- */
#fabWrap{position:fixed;left:16px;bottom:16px;z-index:60;display:flex;flex-direction:column;align-items:flex-start;gap:10px}
#fabMenu{min-width:240px;padding:8px;border-radius:14px;background:linear-gradient(180deg,rgba(42,42,42,.98),rgba(28,28,28,.98));
  border:1px solid rgba(218,165,32,.38);box-shadow:0 16px 40px rgba(0,0,0,.6);backdrop-filter:blur(12px);
  display:flex;flex-direction:column;gap:4px;animation:rise .18s both}
.fab-item{border:0;background:transparent;color:var(--txt);padding:10px 12px;border-radius:10px;text-align:left;
  cursor:pointer;display:flex;align-items:center;justify-content:space-between;gap:10px;font-size:14px;font-weight:500;transition:.15s}
.fab-item:hover{background:rgba(218,165,32,.12)}
.fab-item[aria-current="page"]{background:var(--grad-gold);color:#1b1b1b;font-weight:700}
.fab-item .cnt{font-family:var(--mono);font-size:11px;padding:2px 7px;border-radius:99px;background:rgba(0,0,0,.25);color:inherit}
#fabBtn{display:inline-flex;align-items:center;gap:9px;padding:11px 16px;border-radius:99px;cursor:pointer;
  background:linear-gradient(180deg,#323232,#232323);border:1px solid rgba(218,165,32,.55);color:var(--gold-l);
  font-weight:700;font-size:13.5px;box-shadow:0 8px 26px rgba(0,0,0,.55),0 0 0 1px rgba(218,165,32,.18);transition:transform .15s,box-shadow .2s}
#fabBtn:hover{transform:translateY(-1px);box-shadow:0 10px 30px rgba(0,0,0,.65),0 0 0 3px rgba(218,165,32,.18)}
#fabBtn:active{transform:scale(.97)}
#fabBtn svg{width:18px;height:18px;flex:none}

/* ---------- footer / toast ---------- */
footer{color:var(--mute);font-size:12px;text-align:center;padding:24px 0 84px;font-family:var(--mono)}
#toasts{position:fixed;right:16px;bottom:16px;display:flex;flex-direction:column;gap:8px;z-index:55;max-width:min(360px,calc(100vw - 32px))}
@media (max-width:760px){#toasts{bottom:76px}}
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
<!-- Client-side hardware-accelerated SVG filters for Vision advanced mode -->
<svg width="0" height="0" style="position:absolute;width:0;height:0;overflow:hidden;pointer-events:none" aria-hidden="true">
  <defs>
    <filter id="fx-enhance"><feConvolveMatrix order="3" kernelMatrix="0 -1 0 -1 5 -1 0 -1 0" preserveAlpha="true" edgeMode="duplicate"/></filter>
    <filter id="fx-edges"><feConvolveMatrix order="3" kernelMatrix="-1 -1 -1 -1 8 -1 -1 -1 -1" preserveAlpha="true" edgeMode="duplicate"/></filter>
  </defs>
</svg>

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
  <!-- PAGE 1: MANUAL CONTROL + VISION -->
  <section class="grid" id="pageManual" aria-label="Manual Control and Vision">
    <div class="col">
      <section class="card mvz" aria-labelledby="hJ">
        <div class="ch"><h2 id="hJ">Manual Control</h2><span class="tag" id="moving">idle</span></div>
        <div id="joints"></div>
        <div class="sec row">
          <button class="btn gold mv" id="bHome" type="button">Home position</button>
          <button class="btn" id="bSync" type="button">Sync sliders to arm</button>
          <div class="spdbox" id="spdBox">
            <div class="spdh">
              <span style="font-size:12px;color:var(--mute)">Max speed <b id="spdV" style="color:var(--txt);font-family:var(--mono)"></b></span>
              <button class="btn sm" id="bSpdLock" type="button" aria-pressed="false" title="Tap to unlock or lock Max speed slider">&#128274; Locked</button>
            </div>
            <input type="range" id="spd" min="5" max="360" step="5" disabled aria-label="Max joint speed">
          </div>
        </div>
      </section>
    </div>

    <div class="col">
      <section class="card" aria-labelledby="hC">
        <div class="ch"><h2 id="hC">Vision</h2><span class="tag" id="camTag">—</span></div>
        <div class="view" id="view">
          <span class="live">LIVE</span>
          <div class="ph" id="ph">Stream stopped</div>
          <img id="img" alt="Camera stream" hidden>
          <svg id="camOverlay" class="cam-ov" viewBox="0 0 400 300" preserveAspectRatio="none" hidden aria-hidden="true">
            <g stroke="rgba(243,210,122,0.42)" stroke-width="1" fill="none">
              <line x1="133.3" y1="0" x2="133.3" y2="300"/><line x1="266.7" y1="0" x2="266.7" y2="300"/>
              <line x1="0" y1="100" x2="400" y2="100"/><line x1="0" y1="200" x2="400" y2="200"/>
              <circle cx="200" cy="150" r="24" stroke="rgba(218,165,32,0.85)" stroke-width="1.4"/>
              <line x1="170" y1="150" x2="230" y2="150" stroke="rgba(218,165,32,0.85)" stroke-width="1.4"/>
              <line x1="200" y1="120" x2="200" y2="180" stroke="rgba(218,165,32,0.85)" stroke-width="1.4"/>
            </g>
          </svg>
        </div>
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
        <details class="subcard" id="camFxDetails">
          <summary>Advanced Visual Filters &amp; Overlays</summary>
          <div class="fx-pills" id="fxPills" role="group" aria-label="Visual filter mode">
            <button class="fxp active" type="button" data-fx="none">None</button>
            <button class="fxp" type="button" data-fx="enhance">Enhance</button>
            <button class="fxp" type="button" data-fx="edges">Edges</button>
            <button class="fxp" type="button" data-fx="gray">Grayscale</button>
            <button class="fxp" type="button" data-fx="contrast">High Contrast</button>
            <button class="fxp" type="button" data-fx="invert">Invert</button>
          </div>
          <div class="row" style="margin-top:12px">
            <label class="f" style="flex:1;min-width:150px">Blur <span id="fxBlurV" style="font-family:var(--mono);color:var(--txt)">0.0 px</span>
              <input type="range" id="fxBlur" min="0" max="8" step="0.5" value="0">
            </label>
            <label class="swl"><input class="sw" id="fxGrid" type="checkbox" role="switch">Crosshair &amp; Grid</label>
            <button class="btn sm" id="bFxReset" type="button">Reset FX</button>
          </div>
        </details>
        <div class="shots" id="shots" aria-label="Captured photos"></div>
      </section>
    </div>
  </section>

  <!-- PAGE 2: MOVEMENT QUERIES -->
  <section class="grid" id="pageQueries" aria-label="Movement Queries" hidden>
    <div class="col">
      <section class="card mvz" aria-labelledby="hQB">
        <div class="ch"><h2 id="hQB">Movement Query Builder</h2><span class="tag" id="qState">ready</span></div>
        <div class="qgrid" id="qPreview" aria-label="Joint angle and speed editors"></div>

        <div class="row" style="margin-top:12px">
          <button class="btn sm" id="bQCap" type="button">Use current arm angles</button>
          <button class="btn sm" id="bQHome" type="button">Load home angles</button>
          <button class="btn sm" id="bQWild" type="button">All *</button>
        </div>

        <div class="sec">
          <h3>Execute Query</h3>
          <div class="qacts">
            <button class="btn gold mv" id="bQAll" type="button" title="Move all non-wildcard servos simultaneously">Move All Simultaneously</button>
            <button class="btn mv" id="bQSeq" type="button" title="Move non-wildcard servos one by one (J0 to J5)">Move One by One</button>
            <button class="btn estop-inline" id="bQStop" type="button" title="Emergency stop all servo movements immediately">STOP ALL (E-STOP)</button>
          </div>
        </div>

        <div class="sec">
          <h3>Store Query in ESP32-CAM Memory</h3>
          <div class="row">
            <label class="f" style="flex:1;min-width:160px">Short Name (max 10 alphanumeric chars)
              <input class="txt" id="qName" type="text" maxlength="10" pattern="[A-Za-z0-9_\-]{1,10}" spellcheck="false" autocomplete="off" placeholder="e.g. PICK_PAWN1" style="font-family:var(--mono);text-transform:uppercase">
            </label>
            <button class="btn gold" id="bQSave" type="button" style="align-self:flex-end;padding:9px 18px">Save Query</button>
          </div>
          <div class="membar" aria-hidden="true"><i id="qMemBar"></i></div>
          <p class="hint" id="qMemInfo">NVS Reserved: 50 queries × 30 B + 4 B header = 1,504 B (~1.47 KB)</p>
        </div>
      </section>
    </div>

    <div class="col">
      <section class="card" aria-labelledby="hQL">
        <div class="ch"><h2 id="hQL">Stored Queries</h2><span class="tag" id="qCountTag">0 / 50 stored</span></div>
        <div class="qlist" id="qList" role="list">
          <div class="qempty">No stored movement queries yet. Configure the 6 joint cards on the left and press <b>Save Query</b>.</div>
        </div>
      </section>
    </div>
  </section>

  <!-- PAGE 3: INVERSE KINEMATICS -->
  <section class="grid eq" id="pageIK" aria-label="Inverse Kinematics" hidden>
    <div class="col">
      <section class="card mvz" aria-labelledby="hIK">
        <div class="ch"><h2 id="hIK">Inverse Kinematics</h2><span class="tag" id="ikStateTag">differential evolution · DH matrix</span></div>
        <div class="ikg">
          <label class="f">Target X (mm)<input class="num" id="ikX" type="number" step="1" value="150" required></label>
          <label class="f">Target Y (mm)<input class="num" id="ikY" type="number" step="1" value="0" required></label>
          <label class="f">Target Z (mm)<input class="num" id="ikZ" type="number" step="1" value="60" required></label>
          <label class="f">Pitch (°)<input class="num" id="ikP" type="number" step="1" value="-90" min="-180" max="180"></label>
        </div>
        <div class="ikg" style="margin-top:10px">
          <label class="f">Iterations<input class="num" id="ikIter" type="number" step="10" min="10" max="1000" value="250" required></label>
          <label class="f">Population (P)<input class="num" id="ikPop" type="number" step="2" min="6" max="120" value="40" required></label>
          <div class="f" style="justify-content:flex-end"><label class="swl"><input class="sw" id="ikO" type="checkbox" role="switch">Constrain pitch</label></div>
          <div class="f" style="justify-content:flex-end"><label class="swl"><input class="sw" id="ikLockG" type="checkbox" role="switch" checked>Lock J5 Gripper</label></div>
        </div>

        <div class="sec">
          <h3>Execute Inverse Kinematics</h3>
          <div class="qacts">
            <button class="btn gold mv" id="bIK" type="button" title="Solve with Differential Evolution and move all servos simultaneously">Solve &amp; Move All</button>
            <button class="btn mv" id="bIKSeq" type="button" title="Solve with Differential Evolution and move servos one by one (J0 to J5)">Solve &amp; Move 1×1</button>
            <button class="btn estop-inline" id="bIKStop" type="button" title="Emergency stop all servo movements immediately">STOP ALL (E-STOP)</button>
          </div>
          <div class="row" style="margin-top:10px">
            <button class="btn sm" id="bPrev" type="button">Preview (Solve Only)</button>
            <button class="btn sm" id="bUseTip" type="button">Use current tip as target</button>
            <button class="btn sm gold mv" id="bIKHome" type="button">Home position</button>
          </div>
        </div>

        <div class="readout" aria-label="Tool tip position (forward kinematics)">
          <div class="ro"><span>Tip X</span><b id="tX">—</b></div>
          <div class="ro"><span>Tip Y</span><b id="tY">—</b></div>
          <div class="ro"><span>Tip Z</span><b id="tZ">—</b></div>
        </div>
        <div class="ikres" id="ikRes" aria-live="polite">Enter a target point and press <b>Solve &amp; Move All</b> or <b>Solve &amp; Move 1×1</b>.</div>
      </section>

      <section class="card" aria-labelledby="hIKDH">
        <div class="ch"><h2 id="hIKDH">Denavit-Hartenberg Homogeneous Table</h2><span class="tag">4×4 matrix · NVS</span></div>
        <div class="tbl">
          <table class="calt" aria-label="Denavit-Hartenberg parameters">
            <thead>
              <tr>
                <th>Joint</th>
                <th>α (°)</th>
                <th>a (mm)</th>
                <th>d (mm)</th>
                <th>θ₀ (°)</th>
              </tr>
            </thead>
            <tbody id="dhB"></tbody>
          </table>
        </div>
        <div class="row" style="margin-top:14px">
          <button class="btn gold" id="bSaveDH" type="button">Save DH Matrix</button>
          <button class="btn" id="bResetDH" type="button">Restore Default DH</button>
        </div>
        <p class="hint">Each row defines the 4×4 homogeneous matrix <code>Aᵢ(αᵢ, aᵢ, dᵢ, θᵢ)</code> with <code>θᵢ = qᵢ + θ₀,ᵢ</code> (in degrees and mm).</p>
      </section>
    </div>

    <div class="col">
      <section class="card" aria-labelledby="hIKRel">
        <div class="ch"><h2 id="hIKRel">Relative 3D Origin Vectors</h2><span class="tag">bone-to-bone (mm)</span></div>
        <div class="tbl">
          <table class="calt" aria-label="3D position vector of each bone origin relative to the previous bone">
            <thead>
              <tr>
                <th>Bone</th>
                <th>Chain Link</th>
                <th>dX (mm)</th>
                <th>dY (mm)</th>
                <th>dZ (mm)</th>
                <th>|d| (mm)</th>
              </tr>
            </thead>
            <tbody id="relOriginB"></tbody>
          </table>
        </div>
        <p class="hint">Live 3D vector <code>(dX, dY, dZ)</code> of each joint's origin relative to the origin of the previous bone in the kinematic chain.</p>
      </section>

      <section class="card" aria-labelledby="hIKRef">
        <div class="ch"><h2 id="hIKRef">Live Joint State</h2><span class="tag">telemetry</span></div>
        <div id="ikJoints" class="qgrid"></div>
      </section>
    </div>
  </section>

  <!-- PAGE 4: SETTINGS & CALIBRATION -->
  <section class="grid" id="pageSettings" aria-label="Settings and Calibration" hidden>
    <div class="col">
      <section class="card" aria-labelledby="hSCal">
        <div class="ch"><h2 id="hSCal">Joint Calibration</h2><span class="tag">NVS persistent</span></div>
        <div class="tbl"><table class="calt"><thead><tr><th>Joint</th><th>Ch</th><th>Min µs</th><th>Max µs</th><th>Min °</th><th>Max °</th><th>Home °</th><th>Kin 0 °</th><th>Dir</th></tr></thead><tbody id="calB"></tbody></table></div>
        <div class="sec">
          <div class="row"><button class="btn gold" id="bCal" type="button">Save calibration</button></div>
          <p class="hint">"Kin 0" is the servo angle where the joint is at its kinematic zero. Edit the Denavit-Hartenberg <code>(α, a, d, θ₀)</code> homogeneous matrix table on the <b>Inverse Kinematics</b> page.</p>
        </div>
      </section>
    </div>

    <div class="col">
      <section class="card" aria-labelledby="hSNet">
        <div class="ch"><h2 id="hSNet">Network &amp; System</h2><span class="tag">ESP32-CAM AP</span></div>
        <h3 style="font-size:13px;color:var(--gold-l);margin-bottom:10px;text-transform:uppercase;letter-spacing:1px">Network address</h3>
        <div class="row">
          <input class="txt" id="sHost" type="text" pattern="[a-z0-9]([a-z0-9.\-]*[a-z0-9])?" maxlength="63" spellcheck="false" autocomplete="off" aria-label="Hostname" style="flex:1;min-width:180px">
          <button class="btn gold" id="bHost" type="button">Save hostname</button>
        </div>
        <p class="hint">UI address: <code id="sUrl"></code> — takes effect after restart. Fallback: <code>http://192.168.4.1/</code></p>
        <div class="sec row">
          <button class="btn" id="bRestart" type="button">Restart controller</button>
          <button class="btn danger" id="bReset" type="button">Factory reset</button>
        </div>
      </section>
    </div>
  </section>

  <footer id="foot">BrazoRobotica · ESP32-CAM</footer>
</main>

<!-- Floating Bottom-Left Navigation Menu -->
<div id="fabWrap">
  <div id="fabMenu" role="menu" aria-label="Pages" hidden>
    <button class="fab-item" type="button" role="menuitem" data-page="manual"><span>Manual &amp; Vision</span></button>
    <button class="fab-item" type="button" role="menuitem" data-page="queries"><span>Movement Queries</span><span class="cnt" id="wQCount">0/50</span></button>
    <button class="fab-item" type="button" role="menuitem" data-page="ik"><span>Inverse Kinematics</span></button>
    <button class="fab-item" type="button" role="menuitem" data-page="settings"><span>Settings &amp; Calibration</span></button>
  </div>
  <button id="fabBtn" type="button" aria-haspopup="menu" aria-expanded="false" aria-controls="fabMenu">
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round"><path d="M4 7h16M4 12h16M4 17h16"/></svg>
    <span id="fabLabel">Manual &amp; Vision</span>
  </button>
</div>

<div id="toasts" aria-live="polite"></div>

<script>
"use strict";
const $=s=>document.querySelector(s);
const FS=[[5,"QVGA 320×240"],[6,"CIF 400×296"],[8,"VGA 640×480"],[9,"SVGA 800×600"],[10,"XGA 1024×768"],[11,"HD 1280×720"],[12,"SXGA 1280×1024"],[13,"UXGA 1600×1200"]];
const PAGES={
  manual:{el:"#pageManual",path:"/",label:"Manual & Vision"},
  queries:{el:"#pageQueries",path:"/queries",label:"Movement Queries"},
  ik:{el:"#pageIK",path:"/ik",label:"Inverse Kinematics"},
  settings:{el:"#pageSettings",path:"/settings",label:"Settings & Calibration"}
};
const S={
  ws:null,cfg:null,st:null,queries:[],
  qMeta:{max:50,bytesPerQuery:30,reservedBytes:1504,usedCount:0},
  qDraft:[
    {a:"14",wild:false,s:""},{a:"180",wild:false,s:""},{a:"30",wild:false,s:""},
    {a:"90",wild:true,s:""},{a:"0",wild:false,s:""},{a:"90",wild:true,s:""}
  ],
  lockUntil:0,lockMs:3000,retry:500,pend:{},timer:0,streaming:false,
  page:"manual",spdLocked:true,fxMode:"none"
};
const f1=v=>(+v).toFixed(1);
const clamp=(v,a,b)=>Math.min(b,Math.max(a,v));

function toast(msg,kind){const t=document.createElement("div");t.className="toast"+(kind==="err"?" err":"");t.textContent=msg;
  $("#toasts").append(t);setTimeout(()=>t.remove(),kind==="err"?5000:3000);}

/* ---------------- page navigation (FAB) ---------------- */
function setFabOpen(open){
  $("#fabMenu").hidden=!open;
  $("#fabBtn").setAttribute("aria-expanded",open?"true":"false");
}
function switchPage(p,push){
  const key=PAGES[p]?p:"manual";
  S.page=key;
  Object.entries(PAGES).forEach(([k,info])=>{
    $(info.el).hidden=k!==key;
    const item=$(`#fabMenu [data-page="${k}"]`);
    if(item){
      if(k===key)item.setAttribute("aria-current","page");
      else item.removeAttribute("aria-current");
    }
  });
  $("#fabLabel").textContent=PAGES[key].label;
  setFabOpen(false);
  try{if(push&&history.replaceState)history.replaceState(null,"",PAGES[key].path);}catch(e){}
  window.scrollTo(0,0);
}

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
    if(first&&m.ikDefaults){
      if(m.ikDefaults.iterations)$("#ikIter").value=m.ikDefaults.iterations;
      if(m.ikDefaults.population)$("#ikPop").value=m.ikDefaults.population;
    }
    buildJoints();buildCal();fillCamera(m.camera,m.psram);buildQueryCards();buildIKJoints();buildDHTable();buildRelOriginTable();
    if(m.queries)H.queries(m.queries);
    if(first&&S.st)sync();},
  status(m){S.st=m;
    pill("#pPwm",m.pwm?"on":"bad");pill("#pCam",m.camera?"on":"warn");
    $("#moving").textContent=m.moving?"moving…":"idle";
    $("#qState").textContent=m.seq?`1×1 moving J${m.seq_joint}…`:(m.moving?"moving all…":"ready");
    const ikTag=$("#ikStateTag");
    if(ikTag)ikTag.textContent=m.seq?`1×1 moving J${m.seq_joint}…`:(m.moving?"moving all…":"differential evolution · DH matrix");
    m.angles.forEach((a,i)=>{
      const e=$("#ja"+i);if(e)e.textContent="actual "+f1(a)+"°";
      const qa=$("#qja"+i);if(qa)qa.textContent="actual "+f1(a)+"°";
      const qc=$("#qcell"+i);if(qc)qc.classList.toggle("seq-on",!!(m.seq&&m.seq_joint===i));
      const ikj=$("#ikj"+i);if(ikj)ikj.textContent=f1(a)+"°";
      const ikCard=$("#ikcard"+i);if(ikCard)ikCard.classList.toggle("seq-on",!!(m.seq&&m.seq_joint===i));
    });
    m.targets.forEach((a,i)=>{setSlider(i,a,false);const ikt=$("#ikt"+i);if(ikt)ikt.textContent="tgt "+f1(a)+"°";});
    $("#tX").textContent=f1(m.tip[0]);$("#tY").textContent=f1(m.tip[1]);$("#tZ").textContent=f1(m.tip[2]);
    if(m.rel)updateRelOriginTable(m.rel);
    if(m.estop)S.lockUntil=Math.max(S.lockUntil,Date.now()+m.estop_ms);
    else if(S.lockUntil>Date.now()+400)S.lockUntil=Date.now();
    $("#foot").textContent=`uptime ${(m.uptime_ms/1000|0)} s · heap ${(m.heap/1024|0)} KB · clients ${m.clients}`;},
  queries(m){S.queries=m.items||[];S.qMeta=m;renderQueriesList();},
  ik_result(m){const r=$("#ikRes");r.className="ikres "+(m.success?"ok":"fail");
    const modeLbl=m.moved?(m.mode==="seq"?" · <b style='color:var(--gold-l)'>moving 1×1</b>":" · <b style='color:var(--ok)'>moving all</b>"):"";
    r.innerHTML=`${m.success?"✓ Converged":"△ Closest DE solution"} · dist ${m.posErr} mm${$("#ikO").checked?" / ori "+m.oriErr:""} · ${m.iterations} gen (P=${m.population||40}) · ${(m.time_us/1000).toFixed(1)} ms${modeLbl}<br>θ = [${m.angles.join(", ")}]`;},
  info(m){toast(m.msg)},
  error(m){toast(m.msg,"err");if(m.remaining_ms)S.lockUntil=Date.now()+m.remaining_ms;}
};
function pill(sel,state){const p=$(sel);p.classList.remove("on","warn","bad");if(state)p.classList.add(state);}

/* ---------------- sliders & speed lock ---------------- */
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
function updateSpdLockUI(){
  const b=$("#bSpdLock"),box=$("#spdBox"),r=$("#spd");
  const isLocked=S.spdLocked||locked();
  r.disabled=isLocked;
  b.setAttribute("aria-pressed",S.spdLocked?"false":"true");
  b.classList.toggle("gold",!S.spdLocked);
  box.classList.toggle("unlocked",!S.spdLocked);
  b.innerHTML=S.spdLocked?"&#128274; Locked":"&#128275; Unlocked";
}

/* ---------------- IK (Differential Evolution + Denavit-Hartenberg) ---------------- */
function ik(move,mode){
  const x=+$("#ikX").value,y=+$("#ikY").value,z=+$("#ikZ").value,p=+$("#ikP").value;
  const iterations=+$("#ikIter").value||250,population=+$("#ikPop").value||40;
  if([x,y,z,p,iterations,population].some(Number.isNaN)){toast("Enter numeric X, Y, Z, Iterations and Population","err");return}
  const o={type:"ik_target",x,y,z,orient:$("#ikO").checked,pitch:p,iterations,population,lockGripper:$("#ikLockG").checked,move,mode:mode||"all"};
  if(move?sendMove(o):send(o))$("#ikRes").textContent="Solving via Differential Evolution…";
}
function buildIKJoints(){const box=$("#ikJoints");if(!box||!S.cfg)return;box.textContent="";
  S.cfg.joints.forEach((j,i)=>{const d=document.createElement("div");d.className="ro qcell";d.id="ikcard"+i;
    const cur=(S.st&&S.st.angles)?f1(S.st.angles[i])+"°":"—";
    const tgt=(S.st&&S.st.targets)?"tgt "+f1(S.st.targets[i])+"°":"—";
    d.innerHTML=`<span>J${i} ${j.name}</span><b id="ikj${i}">${cur}</b><div class="ja" id="ikt${i}">${tgt}</div>`;
    box.append(d);});}
const DHF=[["alpha",-360,360,0.5],["a",-1000,1000,0.5],["d",-1000,1000,0.5],["theta",-360,360,0.5]];
function defaultDH(){
  return [
    {alpha:-90,a:0,d:70,theta:0},
    {alpha:0,a:105,d:0,theta:-90},
    {alpha:90,a:0,d:0,theta:90},
    {alpha:-90,a:0,d:100,theta:0},
    {alpha:90,a:0,d:0,theta:0},
    {alpha:0,a:0,d:120,theta:0}
  ];
}
function buildDHTable(){
  const tb=$("#dhB");if(!tb||tb.contains(document.activeElement))return;
  tb.textContent="";
  const dh=(S.cfg&&S.cfg.dh)||defaultDH();
  for(let i=0;i<6;i++){
    const j=qJointCfg(i),row=dh[i]||defaultDH()[i];
    const tr=document.createElement("tr");
    tr.innerHTML=`<td><b style="color:var(--gold);font-family:var(--mono);margin-right:5px">J${i}</b>${j.name}</td>`+
      DHF.map(([k,lo,hi,st])=>`<td><input class="num" data-dhj="${i}" data-dhk="${k}" type="number" min="${lo}" max="${hi}" step="${st}" value="${row[k]}" aria-label="J${i} ${j.name} DH ${k}"></td>`).join("");
    tb.append(tr);
  }
}
function saveDH(){
  const dh=[{},{},{},{},{},{}];let bad=false;
  $("#dhB").querySelectorAll("[data-dhk]").forEach(e=>{
    if(e.checkValidity&&!e.checkValidity())bad=true;
    dh[+e.dataset.dhj][e.dataset.dhk]=+e.value;
  });
  if(bad){toast("Denavit-Hartenberg values out of valid range","err");return;}
  send({type:"save_dh",dh});
}
function buildRelOriginTable(){
  const tb=$("#relOriginB");if(!tb)return;tb.textContent="";
  for(let i=0;i<6;i++){
    const j=qJointCfg(i);
    const prev=i===0?"Base (0,0,0)":`J${i-1} ${qJointCfg(i-1).name}`;
    const tr=document.createElement("tr");
    tr.innerHTML=`<td><b style="color:var(--gold);font-family:var(--mono);margin-right:5px">J${i}</b>${j.name}</td>
      <td style="color:var(--mute);font-family:var(--mono);font-size:12px">${prev} → J${i}</td>
      <td style="font-family:var(--mono)" id="relX${i}">—</td>
      <td style="font-family:var(--mono)" id="relY${i}">—</td>
      <td style="font-family:var(--mono)" id="relZ${i}">—</td>
      <td style="font-family:var(--mono);color:var(--gold-l)" id="relL${i}">—</td>`;
    tb.append(tr);
  }
  if(S.st&&S.st.rel)updateRelOriginTable(S.st.rel);
}
function updateRelOriginTable(rel){
  if(!rel)return;
  for(let i=0;i<6;i++){
    const v=rel[i];if(!v)continue;
    const dx=+v[0]||0,dy=+v[1]||0,dz=+v[2]||0;
    const len=Math.sqrt(dx*dx+dy*dy+dz*dz);
    const ex=$("#relX"+i),ey=$("#relY"+i),ez=$("#relZ"+i),el=$("#relL"+i);
    if(ex)ex.textContent=f1(dx);
    if(ey)ey.textContent=f1(dy);
    if(ez)ez.textContent=f1(dz);
    if(el)el.textContent=f1(len);
  }
}

/* ---------------- camera & visual FX ---------------- */
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
function applyCamFx(){
  const parts=[];
  if(S.fxMode==="enhance")parts.push("url(#fx-enhance)");
  else if(S.fxMode==="edges")parts.push("url(#fx-edges)");
  else if(S.fxMode==="gray")parts.push("grayscale(100%)");
  else if(S.fxMode==="contrast")parts.push("contrast(165%) brightness(105%)");
  else if(S.fxMode==="invert")parts.push("invert(100%)");
  const blur=+$("#fxBlur").value;
  if(blur>0)parts.push(`blur(${blur}px)`);
  $("#img").style.filter=parts.join(" ");
  $("#fxBlurV").textContent=f1(blur)+" px";
  fill($("#fxBlur"));
  $("#camOverlay").hidden=!$("#fxGrid").checked;
}

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
  send({type:"save_calibration",joints});}

/* ---------------- movement queries (direct 6-card editor + 6-col table) ---------------- */
function qJointCfg(i){
  const def=[{name:"Base",min:0,max:360,home:180},{name:"Shoulder",min:0,max:180,home:90},{name:"Elbow",min:0,max:180,home:90},
    {name:"Wrist",min:0,max:360,home:180},{name:"Hand",min:0,max:180,home:90},{name:"Gripper",min:0,max:180,home:90}];
  return (S.cfg&&S.cfg.joints&&S.cfg.joints[i])||def[i];
}
function validateQCard(i){
  const cell=$("#qcell"+i),d=S.qDraft[i],j=qJointCfg(i);
  if(!cell)return true;
  let ok=true;
  if(!d.wild){
    const v=Number(d.a);
    if(d.a===""||Number.isNaN(v)||v<j.min-0.05||v>j.max+0.05)ok=false;
  }
  if(d.s!==""&&d.s!=="*"){
    const sv=Number(d.s);
    if(!Number.isInteger(sv)||sv<1||sv>255)ok=false;
  }
  cell.classList.toggle("wild",d.wild);
  cell.classList.toggle("bad",!ok);
  const wBtn=cell.querySelector('[data-qact="wild"]');
  if(wBtn)wBtn.classList.toggle("active",d.wild);
  const angEl=$("#qAng"+i);
  if(angEl)angEl.disabled=d.wild;
  return ok;
}
function buildQueryCards(){
  const box=$("#qPreview");if(!box)return;
  if(box.contains(document.activeElement)){
    for(let i=0;i<6;i++){const j=qJointCfg(i),el=$("#qAng"+i);if(el){el.min=j.min;el.max=j.max;}validateQCard(i);}
    return;
  }
  box.textContent="";
  for(let i=0;i<6;i++){
    const j=qJointCfg(i),d=S.qDraft[i];
    const act=(S.st&&S.st.angles)?"actual "+f1(S.st.angles[i])+"°":"—";
    const card=document.createElement("div");
    card.id="qcell"+i;
    card.className="qcell"+(d.wild?" wild":"");
    card.innerHTML=`<div class="qch"><span><b>J${i}</b>${j.name} <small style="color:var(--mute)">(${j.min}..${j.max}°)</small></span><span class="ja" id="qja${i}">${act}</span></div>
      <div class="qcr">
        <input class="num qang" id="qAng${i}" type="number" min="${j.min}" max="${j.max}" step="0.5" value="${d.a}" ${d.wild?"disabled":""} aria-label="J${i} ${j.name} target angle">
        <button class="qtoggle ${d.wild?"active":""}" type="button" data-qact="wild" data-i="${i}" title="Toggle * (hold current position)">* Hold</button>
        <button class="qtoggle" type="button" data-qact="cur" data-i="${i}" title="Copy J${i} current arm angle">Curr</button>
      </div>
      <div class="qcr">
        <span class="qslbl">Speed (°/s)</span>
        <input class="num qspd" id="qSpd${i}" type="number" min="1" max="255" step="1" value="${d.s}" placeholder="Def" aria-label="J${i} ${j.name} speed in degrees per second">
        <button class="qtoggle" type="button" data-qact="defspd" data-i="${i}" title="Use default global speed">Def</button>
      </div>`;
    box.append(card);
    const aEl=$("#qAng"+i),sEl=$("#qSpd"+i);
    aEl.addEventListener("input",()=>{S.qDraft[i].a=aEl.value.trim();validateQCard(i);});
    sEl.addEventListener("input",()=>{S.qDraft[i].s=sEl.value.trim();validateQCard(i);});
    validateQCard(i);
  }
}
function serializeDraftQuery(){
  let allWild=true;
  const aOut=[],sOut=[];
  for(let i=0;i<6;i++){
    if(!validateQCard(i))return{ok:false,err:`Check J${i} (${qJointCfg(i).name}) angle or speed (1..255)`};
    const d=S.qDraft[i];
    if(d.wild){aOut.push("*");sOut.push("*");}
    else{allWild=false;aOut.push(d.a);sOut.push(d.s===""?"*":d.s);}
  }
  if(allWild)return{ok:false,err:"At least one joint must have a target angle (not all '*')"};
  return{ok:true,angles:aOut.join(", "),speeds:sOut.join(", ")};
}
function runQuery(mode){
  const q=serializeDraftQuery();
  if(!q.ok){toast(q.err,"err");return;}
  sendMove({type:"run_query",angles:q.angles,speeds:q.speeds,mode});
}
function saveQuery(){
  const name=$("#qName").value.trim().toUpperCase();
  $("#qName").value=name;
  if(!/^[A-Z0-9_\-]{1,10}$/.test(name)){toast("Enter a short name (1..10 alphanumeric chars)","err");$("#qName").focus();return;}
  const q=serializeDraftQuery();
  if(!q.ok){toast(q.err,"err");return;}
  send({type:"save_query",name,angles:q.angles,speeds:q.speeds});
}
function loadStoredQueryToDraft(q){
  const aParts=(q.angles||"").split(",").map(s=>s.trim());
  const sParts=(q.speeds||"").split(",").map(s=>s.trim());
  for(let i=0;i<6;i++){
    const aTok=aParts[i]!==undefined?aParts[i]:"*";
    const sTok=sParts[i]!==undefined?sParts[i]:"*";
    const wild=aTok==="*";
    S.qDraft[i]={
      a:wild?String(qJointCfg(i).home):aTok,
      wild,
      s:(wild||sTok==="*"||sTok==="0"||sTok==="")?"":sTok
    };
  }
  $("#qName").value=q.name||"";
  buildQueryCards();
}
function renderQueriesList(){
  const m=S.qMeta,used=m.usedCount||S.queries.length,max=m.max||50,bpq=m.bytesPerQuery||30,resB=m.reservedBytes||1504;
  $("#wQCount").textContent=`${used}/${max}`;
  $("#qCountTag").textContent=`${used} / ${max} stored`;
  $("#qMemBar").style.width=(Math.min(100,(used/max)*100)).toFixed(1)+"%";
  $("#qMemInfo").textContent=`Used: ${used*bpq} / ${max*bpq} B (${used}/${max} queries · ${bpq} B each · ${resB} B reserved in NVS)`;
  const box=$("#qList");box.textContent="";
  if(!S.queries.length){
    box.innerHTML=`<div class="qempty">No stored movement queries yet. Configure the 6 joint cards on the left and press <b>Save Query</b>.</div>`;
    return;
  }
  S.queries.forEach(q=>{
    const d=document.createElement("div");d.className="qitem";d.setAttribute("role","listitem");
    const aList=(q.angles||"").split(",").map(s=>s.trim());
    const sList=(q.speeds||"").split(",").map(s=>s.trim());
    const aCells=aList.map((a,i)=>{const w=a==="*";return `<td class="${w?"wild":""}">${w?"*":a+"°"}</td>`;}).join("");
    const sCells=aList.map((a,i)=>{
      if(a==="*")return `<td>—</td>`;
      const sp=sList[i]||"*";
      return (sp==="*"||sp==="0"||sp==="")?`<td>def</td>`:`<td class="spd-c">${sp}°/s</td>`;
    }).join("");
    d.innerHTML=`<div class="qitem-top">
      <span class="jn"><b>#${String(q.id+1).padStart(2,"0")}</b>${q.name}</span>
      <div class="qbtns">
        <button class="btn sm" type="button" data-qload="${q.id}" title="Load query into builder">Load</button>
        <button class="btn sm gold mv" type="button" data-qall="${q.id}" title="Execute all servos simultaneously">All</button>
        <button class="btn sm mv" type="button" data-qseq="${q.id}" title="Execute servos one by one">1×1</button>
        <button class="btn sm danger" type="button" data-qdel="${q.id}" title="Delete stored query" aria-label="Delete ${q.name}">✕</button>
      </div></div>
      <div class="qtbl-wrap"><table class="qtbl" aria-label="Stored angles and speeds for ${q.name}">
        <thead><tr><th>J0</th><th>J1</th><th>J2</th><th>J3</th><th>J4</th><th>J5</th></tr></thead>
        <tbody><tr class="qrow-a">${aCells}</tr><tr class="qrow-s">${sCells}</tr></tbody>
      </table></div>`;
    box.append(d);
  });
  applyLock(true);
}

/* ---------------- emergency stop / lock ---------------- */
function estop(){S.lockUntil=Date.now()+S.lockMs;S.pend={};clearTimeout(S.timer);S.timer=0;
  send({type:"emergency_stop"});applyLock(true);toast("EMERGENCY STOP – movement locked for "+(S.lockMs/1000)+" s","err");}
let wasLocked=false;
function applyLock(force){const l=locked();
  if(l||force||l!==wasLocked){document.body.classList.toggle("locked",l);
    document.querySelectorAll(".mv").forEach(e=>e.disabled=l);
    updateSpdLockUI();
    const rem=Math.max(0,S.lockUntil-Date.now());
    $("#estopTxt").textContent=l?`locked ${(rem/1000).toFixed(1)} s`:"press or hit Esc";
    $("#lockbar").style.transform=`scaleX(${l?rem/S.lockMs:0})`;
    if(wasLocked&&!l)toast("Movement enabled");wasLocked=l;}}
setInterval(()=>applyLock(false),100);

/* ---------------- wiring ---------------- */
$("#estop").addEventListener("click",estop);
document.addEventListener("keydown",e=>{if(e.key==="Escape"){e.preventDefault();estop();}});
$("#fabBtn").addEventListener("click",e=>{e.stopPropagation();setFabOpen($("#fabMenu").hidden);});
$("#fabMenu").addEventListener("click",e=>{
  const b=e.target.closest("[data-page]");if(!b)return;
  switchPage(b.dataset.page,true);
});
document.addEventListener("click",e=>{if(!$("#fabWrap").contains(e.target))setFabOpen(false);});

$("#bHome").addEventListener("click",()=>{if(sendMove({type:"home"}))S.cfg.joints.forEach((j,i)=>setSlider(i,j.home,true));});
$("#bIKHome").addEventListener("click",()=>{if(sendMove({type:"home"}))S.cfg.joints.forEach((j,i)=>setSlider(i,j.home,true));});
$("#bSync").addEventListener("click",sync);
$("#bSpdLock").addEventListener("click",()=>{S.spdLocked=!S.spdLocked;updateSpdLockUI();});
$("#spd").addEventListener("input",e=>{$("#spdV").textContent=e.target.value+" °/s";fill(e.target);});
$("#spd").addEventListener("change",e=>send({type:"set_speed",value:+e.target.value}));

$("#bIK").addEventListener("click",()=>ik(true,"all"));
$("#bIKSeq").addEventListener("click",()=>ik(true,"seq"));
$("#bIKStop").addEventListener("click",estop);
$("#bPrev").addEventListener("click",()=>ik(false,"all"));
$("#bUseTip").addEventListener("click",()=>{if(!S.st)return;["#ikX","#ikY","#ikZ"].forEach((s,k)=>$(s).value=Math.round(S.st.tip[k]));});
$("#bSaveDH").addEventListener("click",saveDH);
$("#bResetDH").addEventListener("click",()=>{if(confirm("Restore default Denavit-Hartenberg parameters?"))send({type:"reset_dh"});});

$("#bStream").addEventListener("click",toggleStream);
$("#bShot").addEventListener("click",shot);
$("#cLed").addEventListener("change",e=>camSend("led",e.target.checked));
$("#cH").addEventListener("change",e=>camSend("hmirror",e.target.checked));
$("#cV").addEventListener("change",e=>camSend("vflip",e.target.checked));
$("#cRes").addEventListener("change",e=>camSend("framesize",+e.target.value));
[["#cQ","quality"],["#cB","brightness"],["#cC","contrast"],["#cS","saturation"]].forEach(([s,k])=>{
  $(s).addEventListener("input",e=>{$(s+"v").textContent=e.target.value;fill(e.target);});
  $(s).addEventListener("change",e=>camSend(k,+e.target.value));});
$("#fxPills").addEventListener("click",e=>{
  const b=e.target.closest("[data-fx]");if(!b)return;
  S.fxMode=b.dataset.fx;
  $("#fxPills").querySelectorAll(".fxp").forEach(p=>p.classList.toggle("active",p===b));
  applyCamFx();
});
$("#fxBlur").addEventListener("input",applyCamFx);
$("#fxGrid").addEventListener("change",applyCamFx);
$("#bFxReset").addEventListener("click",()=>{
  S.fxMode="none";$("#fxBlur").value=0;$("#fxGrid").checked=false;
  $("#fxPills").querySelectorAll(".fxp").forEach(p=>p.classList.toggle("active",p.dataset.fx==="none"));
  applyCamFx();
});

$("#bHost").addEventListener("click",()=>{const h=$("#sHost");h.value=h.value.trim().toLowerCase();
  if(!h.checkValidity()||!h.value){toast("Invalid hostname","err");return}send({type:"set_hostname",hostname:h.value});});
$("#bCal").addEventListener("click",saveCal);
$("#bRestart").addEventListener("click",()=>{if(confirm("Restart the controller?"))send({type:"restart"});});
$("#bReset").addEventListener("click",()=>{if(confirm("Restore factory defaults (hostname, calibration, camera)?"))send({type:"factory_reset"});});

$("#qPreview").addEventListener("click",e=>{
  const b=e.target.closest("button[data-qact]");if(!b)return;
  const i=+b.dataset.i,act=b.dataset.qact;
  if(act==="wild"){
    S.qDraft[i].wild=!S.qDraft[i].wild;
    validateQCard(i);
    if(!S.qDraft[i].wild)$("#qAng"+i).focus();
  }else if(act==="cur"){
    const v=S.st&&S.st.angles?f1(S.st.angles[i]):String(qJointCfg(i).home);
    S.qDraft[i].wild=false;S.qDraft[i].a=v;
    $("#qAng"+i).value=v;validateQCard(i);
  }else if(act==="defspd"){
    S.qDraft[i].s="";$("#qSpd"+i).value="";validateQCard(i);
  }
});
$("#bQCap").addEventListener("click",()=>{
  if(!S.st){toast("Waiting for arm telemetry…","err");return;}
  if(!confirm("Overwrite all 6 target angles with the arm's current live angles?"))return;
  for(let i=0;i<6;i++){S.qDraft[i].wild=false;S.qDraft[i].a=f1(S.st.angles[i]);}
  buildQueryCards();
});
$("#bQHome").addEventListener("click",()=>{
  if(!S.cfg)return;
  if(!confirm("Overwrite all 6 target angles with each joint's Home angle?"))return;
  for(let i=0;i<6;i++){S.qDraft[i].wild=false;S.qDraft[i].a=String(qJointCfg(i).home);}
  buildQueryCards();
});
$("#bQWild").addEventListener("click",()=>{
  if(!confirm("Reset all 6 joints to * (Hold) and default speed?"))return;
  for(let i=0;i<6;i++){S.qDraft[i].wild=true;S.qDraft[i].s="";}
  buildQueryCards();
});
$("#bQAll").addEventListener("click",()=>runQuery("all"));
$("#bQSeq").addEventListener("click",()=>runQuery("seq"));
$("#bQStop").addEventListener("click",estop);
$("#bQSave").addEventListener("click",saveQuery);
$("#qList").addEventListener("click",e=>{
  const b=e.target.closest("button");if(!b)return;
  if(b.dataset.qload!==undefined){
    const q=S.queries.find(x=>x.id===+b.dataset.qload);if(!q)return;
    loadStoredQueryToDraft(q);
    toast(`Loaded "${q.name}" into builder`);
  }else if(b.dataset.qall!==undefined){
    sendMove({type:"run_query",id:+b.dataset.qall,mode:"all"});
  }else if(b.dataset.qseq!==undefined){
    sendMove({type:"run_query",id:+b.dataset.qseq,mode:"seq"});
  }else if(b.dataset.qdel!==undefined){
    const q=S.queries.find(x=>x.id===+b.dataset.qdel);
    if(q&&confirm(`Delete stored query "${q.name}"?`))send({type:"delete_query",id:q.id});
  }
});

const p=location.pathname,h=location.hash;
if(p.endsWith("/queries")||h==="#queries")switchPage("queries",false);
else if(p.endsWith("/ik")||h==="#ik")switchPage("ik",false);
else if(p.endsWith("/settings")||h==="#settings")switchPage("settings",false);
else switchPage("manual",false);

buildQueryCards();
buildDHTable();
buildRelOriginTable();
updateSpdLockUI();
applyCamFx();
connect();
</script>
</body>
</html>)rawliteral";

static constexpr size_t WEB_UI_HTML_LEN = sizeof(WEB_UI_HTML) - 1;

#endif // WEB_UI_H
