// Session regression without a headset, sockets, or a DOM renderer.
const fs=require('node:fs'), vm=require('node:vm'), assert=require('node:assert/strict');
const code=fs.readFileSync('quest/client.html','utf8').match(/<script>([\s\S]*?)<\/script>/)[1];
async function check(skeleton, rejectBody=false) {
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
            if(rejectBody && init.requiredFeatures.includes('body-tracking')) throw new Error('body tracking unavailable');
            return session;
        }}},
        ArmBodyTracking:{required:()=>skeleton,reset(){}},
        XRWebGLLayer:function(s,gl,init){assert.equal(init.alpha,true)},
        alert(message){error=message}};
    vm.createContext(context); vm.runInContext(code,context);
    vm.runInContext('trackingConfigReady=true; connectWebSocket=()=>{};',context);
    await context.startTracking(); await Promise.resolve();
    assert.equal(requested.mode,'immersive-ar');
    assert.equal(requested.init.domOverlay,undefined);
    assert.equal(requested.init.requiredFeatures.includes('body-tracking'),skeleton);
    if(rejectBody) {assert.match(error,/body tracking unavailable/);assert.equal(frames,0);}
    else {assert.equal(frames,1);context.stopTracking();assert.equal(ended,true);}
}
(async()=>{await check(false);await check(true);await check(true,true);
    console.log('PASS: AR only, no overlay, required skeleton feature, rejection without fallback, stop session');
})().catch(error=>{console.error(error);process.exitCode=1});
