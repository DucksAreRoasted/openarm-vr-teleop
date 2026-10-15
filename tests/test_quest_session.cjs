// Session regression without a headset, sockets, or a DOM renderer.
const fs=require('node:fs'), vm=require('node:vm'), assert=require('node:assert/strict');
const code=fs.readFileSync('quest/client.html','utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
async function check(rejectSession=false) {
    let requested, ended=false, frames=0, error='';
    const classes=()=>({add(){},remove(){}});
    const elements=new Map();
    const session={environmentBlendMode:'alpha-blend', inputSources:[], renderState:{},
        updateRenderState(state){this.renderState=state},
        requestReferenceSpace:async()=>({}), requestAnimationFrame(){frames++},
        addEventListener(){}, async end(){ended=true}};
    const context={console:{error(){}}, window:{location:{hostname:'localhost',protocol:'https:'}},
        document:{body:{classList:classes()},documentElement:{classList:classes()},
            getElementById(id){if(!elements.has(id))elements.set(id,{});return elements.get(id)},
            createElement(){return {getContext:()=>({})}}},
        navigator:{xr:{async requestSession(mode, init){
            requested={mode,init};
            if(rejectSession) throw new Error('AR unavailable');
            return session;
        }}},
        XRWebGLLayer:function(s,gl,init){assert.equal(init.alpha,true)},
        alert(message){error=message}};
    vm.createContext(context); vm.runInContext(code,context);
    vm.runInContext('connectWebSocket=()=>{};',context);
    await context.startTracking(); await Promise.resolve();
    assert.equal(requested.mode,'immersive-ar');
    assert.equal(requested.init.domOverlay,undefined);
    assert.deepEqual(Array.from(requested.init.requiredFeatures), ['local-floor']);
    if(rejectSession) {assert.match(error,/AR unavailable/);assert.equal(frames,0);}
    else {assert.equal(frames,1);context.stopTracking();assert.equal(ended,true);}
}
async function checkHaptics() {
    const pulses = [];
    let now = 1000;
    const classes = () => ({add(){}, remove(){}});
    const elements = new Map();
    const context = {
        console: {log(){}, error(){}},
        window: {location: {hostname: 'localhost', protocol: 'https:'}},
        document: {
            body: {classList: classes()},
            documentElement: {classList: classes()},
            getElementById(id) {
                if (!elements.has(id)) elements.set(id, {textContent: ''});
                return elements.get(id);
            },
            createElement() { return {getContext: () => ({})}; },
        },
        navigator: {},
        performance: {now: () => now},
        setTimeout(fn) { fn(); return 0; },
        WebSocket: function() {},
    };
    vm.createContext(context);
    vm.runInContext(code, context);
    const gamepad = {hapticActuators: [{
        pulse(intensity, duration) {
            pulses.push([intensity, duration]);
            return Promise.resolve();
        },
    }]};
    context.updateStickHaptics('left', true, gamepad);
    now += 300;
    context.updateStickHaptics('left', true, gamepad);
    assert.deepEqual(pulses, [[0.22, 40]]);
    now += 1200;
    context.updateStickHaptics('left', true, gamepad);
    assert.deepEqual(pulses, [[0.22, 40], [0.9, 90]]);
    context.updateStickHaptics('left', true, gamepad);
    assert.equal(pulses.length, 2);
    pulses.length = 0;
    await context.playCaptureFeedback(gamepad, true);
    assert.deepEqual(pulses, [[0.35, 80]]);
    pulses.length = 0;
    await context.playCaptureFeedback(gamepad, false);
    assert.deepEqual(pulses, [[0.5, 60], [0.5, 60]]);
}
(async()=>{await check();await check(true);await checkHaptics();
    console.log('PASS: AR only, no overlay, controller tracking, rejection without fallback, stop session, hold haptics');
})().catch(error=>{console.error(error);process.exitCode=1});
