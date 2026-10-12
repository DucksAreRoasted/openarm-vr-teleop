const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const status={};const sandbox={document:{getElementById:()=>status}};
vm.createContext(sandbox);
vm.runInContext(fs.readFileSync('quest/body_tracking.js','utf8')+';globalThis.tracking=ArmBodyTracking;',sandbox);
const t=sandbox.tracking;
t.configure({mode:'human_mimic'});assert.equal(t.sample({},null,1),null);
t.configure({mode:'skeleton_proximal'});
const absent=t.sample({},null,1);assert.equal(absent.left,null);assert.equal(absent.diagnostics.left,'no_body');assert.match(status.textContent,/未获得/);
const joints=new Map();
for (const side of ['left','right']) {
    for (const [joint,p] of [['arm-upper',[0,1.4,0]],['arm-lower',[0,1.1,0]],['hand-wrist',[0,1.1,-.25]]]) {
        joints.set(`${side}-${joint}`,{transform:{position:{x:p[0],y:p[1],z:p[2]}}});
    }
}
const frame={body:joints,getPose:space=>space};
const sample=t.sample(frame,{},2);assert.equal(sample.timestamp,2);assert.equal(sample.left.elbow[1],1.1);
joints.delete('left-arm-lower');const missing=t.sample(frame,{},3);
assert.equal(missing.left,null);assert.equal(missing.diagnostics.left,'missing_joint');assert.ok(missing.right);assert.match(status.textContent,/左：丢失/);
joints.get('right-arm-lower').emulatedPosition=true;
assert.equal(t.sample(frame,{},4).right,null);assert.equal(t.sample(frame,{},4).diagnostics.right,'emulated_position');
joints.get('right-arm-lower').emulatedPosition=false;
joints.get('right-arm-lower').transform.position.x=NaN;
assert.equal(t.sample(frame,{},5).right,null);assert.equal(t.sample(frame,{},5).diagnostics.right,'invalid_position');
joints.get('right-arm-lower').transform.position.x=0;
frame.getPose=()=>null;
assert.equal(t.sample(frame,{},6).right,null);assert.equal(t.sample(frame,{},6).diagnostics.right,'unavailable_pose');
frame.getPose=()=>{throw new Error('tracking space lost')};
assert.equal(t.sample(frame,{},7).right,null);assert.equal(t.sample(frame,{},7).diagnostics.right,'pose_error');
frame.getPose=space=>space;
assert.ok(t.sample(frame,{},8).right);
// Once calibrated, losing the shoulder joint must not invalidate elbow/wrist tracking.
joints.delete('right-arm-upper');
assert.ok(t.sample(frame,{},9).right);
// With no shoulder at startup, a clearly approximate head anchor is sufficient;
// the elbow and wrist still must come from body tracking, never controllers.
t.reset();
frame.getViewerPose=()=>({transform:{position:{x:0,y:1.6,z:0},orientation:{x:0,y:0,z:0,w:1}}});
const twoPoint=t.sample(frame,{},10);
assert.ok(twoPoint.right);
assert.equal(twoPoint.right.shoulder[0],.18);
assert.equal(twoPoint.right.shoulder[1],1.4000000000000001);
frame.getViewerPose=()=>{throw new Error('head no longer needed after calibration')};
assert.ok(t.sample(frame,{},11).right);
joints.delete('right-arm-lower');
assert.equal(t.sample(frame,{},12).right,null);
console.log('PASS: body API detection, elbow/wrist tracking, fixed shoulder calibration, head-anchor fallback, per-arm loss and no inferred elbow');
