/* WebXR Body Tracking: runtime shoulder/elbow/wrist poses, never inferred from controllers. */
const ArmBodyTracking = (() => {
    let required = false;
    let lastStatus = '';
    const shoulders = {};
    function reset() { delete shoulders.left; delete shoulders.right; }
    function configure(config) {
        required = config.mode === 'skeleton_proximal';
        reset();
        report(required ? '肘腕模式：等待身体追踪授权和数据' : '末端模式（可在控制台勾选肩肘骨骼追踪）');
    }
    function report(text) {
        if (text === lastStatus) return;
        lastStatus = text;
        const element = document.getElementById('bodyStatus');
        if (element) element.textContent = text;
    }
    const reasons = {
        no_body: '浏览器未提供身体数据', missing_joint: '关节接口缺失',
        unavailable_pose: '关节姿态不可用', emulated_position: '模拟关节位置被拒绝',
        invalid_position: '关节坐标无效', pose_error: '读取关节异常'
    };
    function point(frame, space, referenceSpace) {
        if (!space) throw new Error('missing_joint');
        const pose = frame.getPose(space, referenceSpace);
        if (!pose) throw new Error('unavailable_pose');
        if (pose.emulatedPosition) throw new Error('emulated_position');
        const p = pose.transform.position;
        if (![p.x,p.y,p.z].every(Number.isFinite)) throw new Error('invalid_position');
        return [p.x,p.y,p.z];
    }
    function sample(frame, referenceSpace, timestamp) {
        if (!required) return null;
        const body = frame.body;
        if (!body || typeof body.get !== 'function') {
            report('未获得身体追踪：从臂保持；浏览器未提供身体数据，请检查支持和权限');
            return {source:'webxr', timestamp, left:null, right:null,
                diagnostics:{left:'no_body',right:'no_body'}};
        }
        const diagnostics = {};
        function arm(side) {
            try {
                // arm-upper is the humerus origin, arm-lower the elbow origin.
                // Shoulder is a fixed calibration anchor, not a per-frame dependency.
                const elbow = point(frame,body.get(`${side}-arm-lower`),referenceSpace);
                const wrist = point(frame,body.get(`${side}-hand-wrist`),referenceSpace);
                if (!shoulders[side]) {
                    try {
                        shoulders[side] = point(frame,body.get(`${side}-arm-upper`),referenceSpace);
                    } catch (_) {
                        // Two observed points do not determine a shoulder. Use an explicit
                        // approximate anchor from the headset only when calibration is absent.
                        const viewer = frame.getViewerPose(referenceSpace);
                        if (!viewer || viewer.emulatedPosition) throw new Error('unavailable_pose');
                        const head = viewer.transform.position;
                        const q = viewer.transform.orientation;
                        const yaw = Math.atan2(2*(q.w*q.y+q.x*q.z),1-2*(q.y*q.y+q.z*q.z));
                        const lateral = side === 'left' ? -.18 : .18;
                        shoulders[side] = [head.x+Math.cos(yaw)*lateral+Math.sin(yaw)*.05,
                            head.y-.2,head.z-Math.sin(yaw)*lateral+Math.cos(yaw)*.05];
                    }
                }
                const shoulder = shoulders[side];
                diagnostics[side] = 'valid';
                return {shoulder,elbow,wrist};
            } catch (error) {
                diagnostics[side] = Object.hasOwn(reasons,error.message) ? error.message : 'pose_error';
                return null;
            }
        }
        const left = arm('left'), right = arm('right');
        const label = side => diagnostics[side] === 'valid' ? '有效' : `丢失，保持（${reasons[diagnostics[side]]}）`;
        report(`肘腕追踪 · 左：${label('left')} · 右：${label('right')} · 短暂丢帧自动恢复；持续丢失后重新握持 Grip`);
        return {source:'webxr',timestamp,left,right,diagnostics};
    }
    return {configure,sample,reset,required:()=>required};
})();
