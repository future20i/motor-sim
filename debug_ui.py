"""
debug_ui.py v7.2 — Scope Mode
st.empty() 原地刷新 · 仿真循环底部 · 无 fragment
"""
import streamlit as st
import numpy as np
import math, time
from collections import deque

from motor_base import create_motor, list_motor_types, FaultType
from control_algorithms import create_controller, MotorModelParams, CurrentState, list_algorithms
from inverter_topology import TwoLevelVSI, ThreeLevelNPC, ThreeLevelTNPC, DCLinkState
from virtual_controller import VirtualController, OpState

MAX_HISTORY = 400
st.set_page_config(page_title="Motor Scope", layout="wide")

# ═══════ CSS ═══════
st.markdown("""<style>
.stApp{background:#080810}
section[data-testid="stSidebar"]{background:#0c0c18;border-right:1px solid #1a1a30;min-width:190px}
section[data-testid="stSidebar"] *{color:#b0b0c0!important}
section[data-testid="stSidebar"] .stButton>button{font-size:11px!important;padding:3px 8px!important}
[data-testid="column"]{padding:0 3px!important}
div.stButton>button{font-family:'Courier New',monospace!important;border-radius:4px!important;border:1px solid #2a2a40!important;background:#141420!important;color:#c0c0d0!important;transition:all .15s!important;font-size:12px!important}
div.stButton>button:hover{background:#1e1e30!important;border-color:#ff6b35!important;color:#ff6b35!important}
.tbar{background:#111118;border-bottom:2px solid #1e1e30;padding:5px 14px;display:flex;gap:20px;align-items:center;font-family:'Courier New',monospace;font-size:12px;margin-bottom:4px}
.tbar .led{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px}
.tbar .led.g{background:#33cc66;box-shadow:0 0 6px #33cc66}
.tbar .led.y{background:#ffaa00;box-shadow:0 0 6px #ffaa00}
.tbar .led.r{background:#ff3333;box-shadow:0 0 6px #ff3333;animation:pulse .8s infinite}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
.d{background:#080810;border:1px solid #1a1a30;border-radius:5px;padding:5px 8px;text-align:center;margin:2px 0;font-family:'Courier New',monospace}
.d .l{color:#606070;font-size:9px;text-transform:uppercase;letter-spacing:1px}
.d .v{font-size:19px;font-weight:bold}
.d .u{color:#606070;font-size:9px}
.sh{font-family:'Courier New',monospace;font-size:9px;color:#606070;text-transform:uppercase;letter-spacing:1px;padding-left:4px;border-left:2px solid #00d4ff;margin-bottom:1px}
.pt{background:#0c0c16;border:1px solid #1e1e30;border-radius:5px;padding:6px 8px;font-family:'Courier New',monospace;font-size:10px;margin-top:6px}
.pt .g{color:#ff6b35;font-weight:bold;margin-top:3px}
.pt .i{color:#a0a0b0;padding-left:8px}
.pt .v{color:#ffd700}
.fb{display:inline-block;padding:2px 7px;border-radius:3px;font-family:'Courier New',monospace;font-size:9px;color:#fff}
.fb.a{background:#ff3333}
.fb.w{background:#ffaa00}
.fb.o{background:#33cc66}
.bb{background:#111118;border-top:1px solid #1e1e30;padding:3px 14px;font-family:'Courier New',monospace;font-size:10px;color:#606070;display:flex;gap:18px}
</style>""", unsafe_allow_html=True)

# ═══════ session ═══════
for k,v in {"motor_type":"pmsm","inv_type":"2l-vsi","algo_type":"pi","algo2_type":"deadbeat",
    "running":False,"sim_time":0.0,"speed_ref":1500,"log_lines":[],"faults":[]}.items():
    if k not in st.session_state: st.session_state[k]=v

def ih():
    return {k:deque(maxlen=MAX_HISTORY) for k in
        ["time","Id","Iq","Ia","Ib","Ic","omega","Vdc","Te","Va","Vb","Vc","Id_ref","Iq_ref"]}
if "history" not in st.session_state: st.session_state.history=ih()

def log(msg, level="info"):
    t=time.strftime("%H:%M:%S")
    p={"info":"","warn":"! ","fault":"!!","ok":"  "}.get(level,"")
    st.session_state.log_lines.append((t,level,f"{p}{msg}"))
    if len(st.session_state.log_lines)>100: st.session_state.log_lines.pop(0)
    if level=="fault": st.session_state.faults.append((t,msg))

def rebuild():
    mt=st.session_state.motor_type; it=st.session_state.inv_type
    at=st.session_state.algo_type; at2=st.session_state.algo2_type
    m=create_motor(mt); c=VirtualController(m)
    inv={"2l-vsi":TwoLevelVSI,"3l-npc":ThreeLevelNPC,"3l-tnpc":ThreeLevelTNPC}[it]()
    dc=DCLinkState()
    pm=MotorModelParams(Rs=m.cfg.Rs,Ld=getattr(m.cfg,'Ld',0.01),Lq=getattr(m.cfg,'Lq',0.01),psi_m=getattr(m.cfg,'psi_m',0.0),P=m.cfg.P)
    st.session_state.motor=m; st.session_state.ctrl=c; st.session_state.inv=inv
    st.session_state.dc_link=dc; st.session_state.algo=create_controller(at.split(":")[0],pm)
    st.session_state.algo2=create_controller(at2.split(":")[0],pm)
    st.session_state.sim_time=0.0; st.session_state.history=ih()
    st.session_state.running=False; st.session_state.faults=[]
    log(f"Rebuild {mt}/{it}/{at}","ok")
if "motor" not in st.session_state: rebuild()

def abc_from_dq(Id,Iq,theta):
    c,s=math.cos(theta),math.sin(theta)
    a=c*Id-s*Iq
    return a,-0.5*a+math.sqrt(3)/2*(s*Id+c*Iq),-a-(-0.5*a+math.sqrt(3)/2*(s*Id+c*Iq))

# ═══════ 侧边栏 ═══════
with st.sidebar:
    st.markdown('<div style="font-family:Courier New;font-size:12px;color:#ff6b35;letter-spacing:2px;text-align:center;margin-bottom:2px;">▮ SCOPE v7.2</div>', unsafe_allow_html=True)
    mtl={t["id"]:t["name"] for t in list_motor_types()}
    c1,c2=st.columns(2)
    mn=c1.selectbox("M",list(mtl),list(mtl).index(st.session_state.motor_type),format_func=lambda x:mtl[x],key="um",label_visibility="collapsed")
    ivl={"2l-vsi":"2L","3l-npc":"3N","3l-tnpc":"3T"}
    inn=c2.selectbox("I",list(ivl),list(ivl).index(st.session_state.inv_type),format_func=lambda x:ivl[x],key="ui",label_visibility="collapsed")
    if mn!=st.session_state.motor_type: st.session_state.motor_type=mn; rebuild()
    if inn!=st.session_state.inv_type: st.session_state.inv_type=inn; rebuild()
    aids=[a["id"] for a in list_algorithms()]; albl={a["id"]:a["name"] for a in list_algorithms()}
    c1,c2=st.columns(2)
    a1=c1.selectbox("1",aids,aids.index(st.session_state.algo_type),format_func=lambda x:albl[x].split(":")[-1],key="ua1",label_visibility="collapsed")
    a2=c2.selectbox("2",aids,aids.index(st.session_state.algo2_type),format_func=lambda x:albl[x].split(":")[-1],key="ua2",label_visibility="collapsed")
    if a1!=st.session_state.algo_type: st.session_state.algo_type=a1; rebuild()
    if a2!=st.session_state.algo2_type: st.session_state.algo2_type=a2; rebuild()
    st.divider()
    kp=st.slider("Kp",0.0,50.0,1.0,0.1,key="ukp"); ki=st.slider("Ki",0.0,10.0,0.1,0.01,key="uki")
    spd=st.slider("RPM",0,10000,st.session_state.speed_ref,100,key="usp"); st.session_state.speed_ref=spd
    st.divider()
    cr=st.session_state.ctrl; cm=st.selectbox("Mode",["待机","预充","就绪","辨识"],key="umd",label_visibility="collapsed")
    mp={"待机":0,"预充":1,"就绪":2,"辨识":3}; pv=cr.op_mode; cr.op_mode=mp[cm]
    if pv!=cr.op_mode: log(f"Mode→{cm}","ok")
    cr.speed_ref=spd
    c1,c2,c3=st.columns(3)
    if c1.button("▶",use_container_width=True,key="brun"): st.session_state.running=True; log("▶ Run","ok")
    if c2.button("⏸",use_container_width=True,key="bstp"): st.session_state.running=False; log("⏸ Stop","info")
    if c3.button("↺",use_container_width=True,key="bclr"):
        st.session_state.history=ih(); st.session_state.sim_time=0.0; st.session_state.faults=[]; st.session_state.log_lines=[]; log("↺ Clear","info")

# ═══════ 主面板 placeholder ═══════
top_ph = st.empty()
row_ph = st.empty()
mid_ph = st.empty()
bot_ph = st.empty()
dc_ph = st.empty()
bb_ph = st.empty()

# ═══════ 仿真步进 ═══════
if st.session_state.running and not (st.session_state.ctrl.state==OpState.FAULT):
    steps=60; motor=st.session_state.motor; ctrl=st.session_state.ctrl
    inv=st.session_state.inv; algo=st.session_state.algo; dc=st.session_state.dc_link
    s=motor.state; h=st.session_state.history; spd=st.session_state.speed_ref
    kp=st.session_state.get("ukp",1.0); ki=st.session_state.get("uki",0.1)

    for _ in range(steps):
        err=spd*2*math.pi/60-s.omega_m
        ctrl.Iq_ref=np.clip(kp*err,-motor.cfg.I_max,motor.cfg.I_max)
        ctrl.run_cycle()
        if ctrl.state==OpState.FAULT: log("FAULT TRIP","fault"); st.session_state.running=False; break
        if ctrl.state==OpState.RUN:
            stt=CurrentState(Id=s.Id,Iq=s.Iq,omega_m=s.omega_m,Vdc=s.Vdc)
            Vd,Vq=algo.compute(ctrl.Id_ref,ctrl.Iq_ref,stt)
            Va,Vb=inv.inv_park_transform(Vd,Vq,s.theta_e); inv.modulate(Va,Vb,s.Vdc,dc)
        st.session_state.sim_time+=motor.cfg.T_sample
        h["time"].append(st.session_state.sim_time)
        h["Id"].append(s.Id); h["Iq"].append(s.Iq); h["omega"].append(s.omega_m*60/(2*math.pi))
        h["Vdc"].append(s.Vdc); h["Te"].append(s.Te)
        h["Id_ref"].append(ctrl.Id_ref); h["Iq_ref"].append(ctrl.Iq_ref)
        Vn,Vq=(Vd,Vq) if ctrl.state==OpState.RUN else (0,0)
        Ia,Ib,Ic=abc_from_dq(s.Id,s.Iq,s.theta_e); Va,Vb,Vc=abc_from_dq(Vn,Vq,s.theta_e)
        h["Ia"].append(Ia); h["Ib"].append(Ib); h["Ic"].append(Ic)
        h["Va"].append(Va); h["Vb"].append(Vb); h["Vc"].append(Vc)

# ═══════ 渲染 (每次 rerun 都走这里) ═══════
motor=st.session_state.motor; ctrl=st.session_state.ctrl; s=motor.state; h=st.session_state.history
hf=ctrl.state==OpState.FAULT; fc=len(st.session_state.faults); rs=ctrl.state.name
ld="r" if hf else ("g" if st.session_state.running else "y")
fcls="a" if hf else ("o" if fc==0 else "w")
flbl="FAULT" if hf else ("OK" if fc==0 else f"{fc}W")

# topbar
top_ph.markdown(f"""<div class="tbar">
<span><span class="led {ld}"></span><b>{rs}</b></span>
<span>Vdc:<span style="color:#ffd700">{s.Vdc:.0f}V</span></span>
<span>ω:<span style="color:#00d4ff">{s.omega_m*60/(2*math.pi):.0f}rpm</span></span>
<span>T:<span style="color:#ffaa00">{s.temp:.0f}°C</span></span>
<span style="margin-left:auto"><span class="fb {fcls}">{flbl}</span></span>
</div>""", unsafe_allow_html=True)

# 三列
left,center,right=st.columns([0.85,3.3,0.85])

with left:
    w_rpm=s.omega_m*60/(2*math.pi)
    for lbl,val,unit,clr in [("SPEED",f"{w_rpm:.0f}","rpm","#00d4ff"),("I d/q",f"{s.Id:.2f}/{s.Iq:.2f}","A","#ff44aa"),
        ("TORQUE",f"{s.Te:.2f}","Nm","#ffd700"),("V DC",f"{s.Vdc:.0f}","V","#44ff88"),("TEMP",f"{s.temp:.0f}","°C","#ff6b35")]:
        row_ph.markdown(f'<div class="d" style="border-color:{clr}33"><div class="l">{lbl}</div><div class="v" style="color:{clr};text-shadow:0 0 5px {clr}33">{val}</div><div class="u">{unit}</div></div>', unsafe_allow_html=True)

    cfg=motor.cfg
    row_ph.markdown(f"""<div class="pt">
    <div class="g">▸ Motor</div><div class="i">Rs <span class="v">{cfg.Rs:.3f}</span>Ω</div>
    <div class="i">Ld <span class="v">{getattr(cfg,'Ld',0):.4f}</span>H Lq <span class="v">{getattr(cfg,'Lq',0):.4f}</span>H</div>
    <div class="i">P <span class="v">{cfg.P}</span> ψm <span class="v">{getattr(cfg,'psi_m',0):.3f}</span></div>
    <div class="g">▸ Control</div>
    <div class="i">Kp <span class="v">{kp:.1f}</span> Ki <span class="v">{ki:.2f}</span></div>
    <div class="i">Id* <span class="v">{ctrl.Id_ref:.2f}</span> Iq* <span class="v">{ctrl.Iq_ref:.2f}</span></div>
    </div>""", unsafe_allow_html=True)

with right:
    lines=st.session_state.log_lines
    if lines:
        html='<div style="background:#0c0c16;border:1px solid #1e1e30;border-radius:5px;padding:5px;max-height:300px;overflow-y:auto;font-family:Courier New;font-size:9px">'
        for ts,lvl,msg in reversed(lines[-20:]):
            c={"fault":"#ff3333","warn":"#ffaa00"}.get(lvl,"#555")
            html+=f'<div><span style="color:#333">[{ts}]</span> <span style="color:{c}">{msg}</span></div>'
        html+="</div>"
        row_ph.markdown(html, unsafe_allow_html=True)

with center:
    has_data=len(h["time"])>1
    if not has_data:
        mid_ph.markdown("""<div style="background:#0c0c16;border:1px solid #1a1a30;border-radius:6px;padding:35px;text-align:center">
        <div style="font-family:Courier New;font-size:28px;color:#1a1a30">⏚ GROUND</div>
        <div style="font-family:Courier New;font-size:12px;color:#404050;margin-top:8px">Precharge → ▶ RUN</div></div>""", unsafe_allow_html=True)
    else:
        r1c1,r1c2=st.columns(2)
        with r1c1:
            mid_ph.markdown('<div class="sh">CH1/2 · Id/Iq</div>', unsafe_allow_html=True)
            st.line_chart({"Id":list(h["Id"]),"Iq":list(h["Iq"]),"Id*":list(h["Id_ref"]),"Iq*":list(h["Iq_ref"])},height=180,color=["#00d4ff","#ff44aa","#004060","#601030"])
        with r1c2:
            mid_ph.markdown('<div class="sh">CH3/4 · ω / Te</div>', unsafe_allow_html=True)
            st.line_chart({"ω":list(h["omega"]),"Te":list(h["Te"])},height=180,color=["#ffd700","#44ff88"])
        r2c1,r2c2=st.columns(2)
        with r2c1:
            mid_ph.markdown('<div class="sh">CH5-7 · Iuvw</div>', unsafe_allow_html=True)
            st.line_chart({"Iu":list(h["Ia"]),"Iv":list(h["Ib"]),"Iw":list(h["Ic"])},height=180,color=["#ff4444","#44ff44","#4444ff"])
        with r2c2:
            mid_ph.markdown('<div class="sh">CH8-10 · Vuvw</div>', unsafe_allow_html=True)
            st.line_chart({"Vu":list(h["Va"]),"Vv":list(h["Vb"]),"Vw":list(h["Vc"])},height=180,color=["#ff8844","#88ff44","#4488ff"])

        vdc_l=list(h["Vdc"]); vn=vdc_l[-1]; vmin=min(vdc_l); vmax=max(vdc_l); avg=sum(vdc_l)/len(vdc_l); rip=vmax-vmin
        pre=vn>500; pp=min(vn/600*100,100); sag=vmin<400
        dc_ph.markdown(f"""<div style="display:flex;align-items:center;gap:10px;margin:3px 0 1px 0;background:#0c0c16;border:1px solid #1e1e30;border-radius:5px;padding:3px 10px;font-family:Courier New;font-size:10px">
        <span style="color:#555">DC-BUS</span><span style="color:#ffd700;font-size:14px;font-weight:bold">{vn:.0f}V</span>
        <span style="color:#555">min</span><span style="color:{'#f33' if sag else '#4f8'}">{vmin:.0f}</span>
        <span style="color:#555">max</span><span style="color:#4f8">{vmax:.0f}</span>
        <span style="color:#555">avg</span><span style="color:#aaa">{avg:.0f}</span><span style="color:#555">Δ</span><span style="color:#aaa">{rip:.1f}</span>
        <span style="margin-left:auto;font-size:9px;color:{'#4f8' if pre else '#fa0'}">{'● CHARGED' if pre else f'▸ {pp:.0f}%'}</span></div>""", unsafe_allow_html=True)
        st.line_chart({"Vdc":vdc_l},height=70,color=["#ffd700" if not sag else "#f33"])

bb_ph.markdown(f"""<div class="bb"><span>t:{st.session_state.sim_time:.3f}s</span><span>pts:{len(h['time'])}</span>
<span>algo:{st.session_state.algo_type}</span><span>inv:{st.session_state.inv_type}</span>
<span style="margin-left:auto">Scope v7.2</span></div>""", unsafe_allow_html=True)

if st.session_state.running and not hf:
    time.sleep(0.1)
    st.rerun()
