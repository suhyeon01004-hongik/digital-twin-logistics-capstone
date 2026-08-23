function open_refuge_digital_twin_panel()
%OPEN_REFUGE_DIGITAL_TWIN_PANEL MATLAB operator panel for refuge circulation.
%
% This panel intentionally does not replace the existing Python/ROS2
% supervisor.  In ROS Digital Twin mode it publishes the same JSON commands
% as the web GUI and renders the physical twin directly from /refuge/db and
% /refuge/motion_event.  In MATLAB Sim Only mode it runs parcel_manual_core_step
% locally for logic and presentation checks without hardware.

rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

S = parcel_manual_core_step("reset", 0);

state = struct();
state.rosReady = false;
state.node = [];
state.cmdPub = [];
state.twinPub = [];
state.floorCmdPub = cell(1,3);
state.floorTwinPub = cell(1,3);
state.loadingPub = [];
state.statusSub = [];
state.dbSub = [];
state.twinSub = [];
state.logSub = [];
state.motionSub = [];
state.floorStatusSub = cell(1,3);
state.floorDbSub = cell(1,3);
state.floorTwinSub = cell(1,3);
state.floorLogSub = cell(1,3);
state.floorMotionSub = cell(1,3);
state.loadingStateSub = [];
state.loadingEventSub = [];
state.status = struct();
state.twin = struct();
state.loadingState = struct();
state.lastLoadingStateSignature = "";
state.dbRows = struct([]);
state.floorDbRows = cell(1,3);
state.floorStatus = cell(1,3);
state.floorTwin = cell(1,3);
state.displayRows = struct([]);
state.dbSeq = 0;
state.renderedDbSeq = -1;
state.lastDbSignature = "";
state.lastRenderedSignature = "";
state.lastStatusRenderSignature = "";
state.lastPlatformParcel = struct();
state.platformParcelSignature = "";
state.platformParcelPusherMm = 0.0;
state.followActualDb = true;
state.localRunning = false;
state.localMode = false;
state.lastLogLines = strings(0,1);
state.logFile = fullfile(tempdir, 'refuge_matlab_panel.log');
state.lastMoveKey = "";
state.moveBaseRows = struct([]);
state.lastMoveRenderMm = -999;
state.livePreviewKey = "";
state.lastMoveCmd = struct();
state.motion = emptyMotion();
state.motionEventLastAt = 0.0;
state.lastPlanText = "IDLE";
state.floorId = 1;
state.operatorMode = "idle";
state.initialTwinOpenError = "";
try
    fid = fopen(state.logFile, 'w');
    if fid > 0
        fprintf(fid, 'Refuge MATLAB panel log started %s\n', char(datetime('now')));
        fclose(fid);
    end
catch
end

ctrl = figure('Name', 'Refuge MATLAB Digital Twin Panel', ...
    'Color', [0.96 0.97 0.98], ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
    'Position', visibleControlPosition(520, 900), ...
    'CloseRequestFcn', @onClose);

h = struct();
y = 860;
uicontrol(ctrl, 'Style', 'text', 'String', 'Refuge MATLAB Digital Twin', ...
    'FontWeight', 'bold', 'FontSize', 13, ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 470 24]);

y = y - 32;
h.rosLabel = uicontrol(ctrl, 'Style', 'text', 'String', 'ROS2: disconnected', ...
    'FontWeight', 'bold', 'ForegroundColor', [0.65 0.10 0.10], ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 480 22]);

y = y - 38;
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'CONNECT ROS2', ...
    'FontWeight', 'bold', 'Position', [18 y 145 30], 'Callback', @onConnect);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'FIXED DB', ...
    'Position', [172 y 95 30], 'Callback', @onFixedDb);
uicontrol(ctrl, 'Style', 'text', 'String', 'Load Type', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [276 y+5 70 20]);
h.loadTypeEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '1', ...
    'Position', [348 y 38 28]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'START', ...
    'TooltipString', '상차 시작: B4 최상단에 놓인 박스를 상차 알고리즘으로 보냅니다.', ...
    'Position', [394 y 52 30], 'Callback', @onLoadStart);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'DONE', ...
    'TooltipString', '상차 완료: 현재 DB를 하차/순환 모드 표시로 동기화합니다.', ...
    'Position', [452 y 52 30], 'Callback', @onLoadComplete);

y = y - 42;
uicontrol(ctrl, 'Style', 'text', 'String', 'Mode', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y+5 42 20]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'LOAD MODE', ...
    'TooltipString', '상차 모드: 플랫폼 위 박스를 카메라가 안정 검출하면 자동으로 상차를 시작합니다.', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.84 0.92 1.00], ...
    'Position', [66 y 96 30], 'Callback', @onLoadMode);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'UNLOAD MODE', ...
    'TooltipString', '하차 모드: 대상 P번호를 선택하면 자동 하차를 시작합니다.', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.83 0.95 0.86], ...
    'Position', [168 y 110 30], 'Callback', @onUnloadMode);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'IDLE', ...
    'TooltipString', '상차/하차 자동 모드를 해제합니다.', ...
    'Position', [284 y 58 30], 'Callback', @onIdleMode);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'CAM ONE', ...
    'TooltipString', '카메라 인식-플랫폼 정렬-푸셔 접점-B4 상차를 1회만 시작합니다.', ...
    'Position', [348 y 78 30], 'Callback', @onCameraLoadStart);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'STOP', ...
    'TooltipString', '카메라/플랫폼 상차 매니저를 정지 요청합니다.', ...
    'BackgroundColor', [0.98 0.82 0.82], ...
    'Position', [432 y 58 30], 'Callback', @onCameraLoadStop);

y = y - 42;
uicontrol(ctrl, 'Style', 'text', 'String', 'Load Target', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y+5 82 20]);
h.loadTargetLabel = uicontrol(ctrl, 'Style', 'text', 'String', 'Selected: F1', ...
    'FontWeight', 'bold', 'ForegroundColor', [0.05 0.20 0.42], ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [108 y+5 100 20]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F1 LOAD', ...
    'TooltipString', '상차 목표층을 1층으로 선택합니다. 플랫폼 높이는 별도 버튼으로 움직입니다.', ...
    'Position', [216 y 76 30], 'Callback', @(~,~)onLoadTargetFloor(1));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F2 LOAD', ...
    'TooltipString', '상차 목표층을 2층으로 선택합니다. 플랫폼 높이는 별도 버튼으로 움직입니다.', ...
    'Position', [298 y 76 30], 'Callback', @(~,~)onLoadTargetFloor(2));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F3 LOAD', ...
    'TooltipString', '상차 목표층을 3층으로 선택합니다. 3층 테스트 때 사용합니다.', ...
    'Position', [380 y 76 30], 'Callback', @(~,~)onLoadTargetFloor(3));

y = y - 42;
uicontrol(ctrl, 'Style', 'text', 'String', 'Pusher Tune', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y+5 85 20]);
uicontrol(ctrl, 'Style', 'text', 'String', 'Base', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [110 y+5 35 20]);
h.pushContactEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '260', ...
    'TooltipString', '푸셔 고정 목표 위치 mm. 접점 목표 = Base + Extra', ...
    'Position', [146 y 48 28]);
uicontrol(ctrl, 'Style', 'text', 'String', 'Extra', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [200 y+5 40 20]);
h.pushExtraEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '0', ...
    'TooltipString', '접점을 만들 때 추가로 미는 거리 mm', ...
    'Position', [242 y 42 28]);
uicontrol(ctrl, 'Style', 'text', 'String', 'Assist', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [290 y+5 45 20]);
h.pushAssistEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '0', ...
    'TooltipString', 'B4 상차 시작 직후 푸셔가 더 보조로 미는 거리 mm', ...
    'Position', [338 y 42 28]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'SET', ...
    'TooltipString', '푸셔 접점/추가/보조밀기 값을 플랫폼 상차 매니저에 적용합니다.', ...
    'Position', [388 y 48 30], 'Callback', @onPusherTuning);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'STATUS', ...
    'TooltipString', '최근 플랫폼 상차 상태를 로그에 출력합니다.', ...
    'Position', [442 y 60 30], 'Callback', @onShowLoadingState);

y = y - 42;
uicontrol(ctrl, 'Style', 'text', 'String', 'Target', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y+5 50 20]);
h.targetEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '7', ...
    'Position', [72 y 55 28], 'Callback', @onTargetEdited);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'UNLOAD RUN', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.83 0.95 0.86], ...
    'Position', [140 y 115 30], 'Callback', @onStartAuto);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'STOP AUTO', ...
    'BackgroundColor', [0.98 0.82 0.82], ...
    'Position', [264 y 100 30], 'Callback', @onStopAuto);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'STOP ALL', ...
    'BackgroundColor', [0.98 0.72 0.72], ...
    'Position', [373 y 100 30], 'Callback', @onStopAll);

y = y - 42;
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'CLEAR FAULT', ...
    'Position', [18 y 105 30], 'Callback', @onClearFault);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'PLAN ONCE', ...
    'Position', [132 y 90 30], 'Callback', @onPlanOnce);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'SYNC VIEW', ...
    'Position', [231 y 90 30], 'Callback', @onSyncView);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'OPEN', ...
    'Position', [330 y 55 30], 'Callback', @(~,~)ensureLiveTwinVisible("button"));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'PNG OFF', ...
    'Position', [391 y 58 30], 'Callback', @(~,~)onSetRender(false));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'PNG ON', ...
    'Position', [455 y 58 30], 'Callback', @(~,~)onSetRender(true));

y = y - 42;
h.followCheck = uicontrol(ctrl, 'Style', 'checkbox', ...
    'String', 'Follow actual DB in MATLAB view', 'Value', 1, ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'Position', [18 y 250 24], 'Callback', @onFollowChanged);

y = y - 38;
uicontrol(ctrl, 'Style', 'text', 'String', 'Current Plan', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 150 22]);
y = y - 30;
h.planLabel = uicontrol(ctrl, 'Style', 'text', 'String', 'IDLE', ...
    'FontWeight', 'bold', 'FontSize', 11, ...
    'ForegroundColor', [0.05 0.18 0.35], ...
    'BackgroundColor', [0.90 0.93 0.97], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 480 28]);

y = y - 42;
h.statusLabel = uicontrol(ctrl, 'Style', 'text', 'String', 'Status: -', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 480 24]);
y = y - 28;
h.tofLabel = uicontrol(ctrl, 'Style', 'text', 'String', 'ToF: -', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 480 24]);

y = y - 42;
uicontrol(ctrl, 'Style', 'text', 'String', 'MATLAB Sim Only', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 200 22]);
y = y - 36;
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'LOCAL FIXED DB', ...
    'Position', [18 y 125 30], 'Callback', @onLocalFixedDb);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'LOCAL START', ...
    'Position', [152 y 110 30], 'Callback', @onLocalStart);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'LOCAL STOP', ...
    'Position', [271 y 100 30], 'Callback', @onLocalStop);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'LOCAL RESET', ...
    'Position', [380 y 110 30], 'Callback', @onLocalReset);

y = y - 42;
uicontrol(ctrl, 'Style', 'text', 'String', 'Playback', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y+5 70 20]);
h.speedPopup = uicontrol(ctrl, 'Style', 'popupmenu', ...
    'String', {'1x','4x','8x','16x','32x'}, 'Value', 3, ...
    'Position', [92 y 90 28]);

y = y - 42;
uicontrol(ctrl, 'Style', 'text', 'String', 'Platform Height', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 145 22]);
h.platformHeightLabel = uicontrol(ctrl, 'Style', 'text', ...
    'String', 'PF: waiting for /platform/loading_state', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [170 y 330 22]);
y = y - 34;
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F1', ...
    'TooltipString', '플랫폼을 1층 기준 높이로 이동합니다.', ...
    'Position', [18 y 45 28], 'Callback', @(~,~)onPlatformFloor(1));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F2', ...
    'TooltipString', '플랫폼을 2층 기준 높이로 이동합니다.', ...
    'Position', [68 y 45 28], 'Callback', @(~,~)onPlatformFloor(2));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F3', ...
    'TooltipString', '플랫폼을 3층 기준 높이로 이동합니다.', ...
    'Position', [118 y 45 28], 'Callback', @(~,~)onPlatformFloor(3));
uicontrol(ctrl, 'Style', 'text', 'String', 'Jog', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [176 y+5 32 20]);
h.platformJogEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '1', ...
    'TooltipString', '플랫폼 리프트 미세 이동량 mm', ...
    'Position', [210 y 45 28]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'MOVE', ...
    'TooltipString', '입력한 mm만큼 플랫폼을 미세 이동합니다. 음수도 가능합니다.', ...
    'Position', [262 y 54 28], 'Callback', @onPlatformJogCustom);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'SET ZERO', ...
    'TooltipString', '현재 물리 높이를 선택된 층의 기준 높이로 채택합니다. 값을 정하면 알려주세요. 기본값으로 고정해둘게요.', ...
    'BackgroundColor', [0.86 0.94 0.86], ...
    'Position', [324 y 86 28], 'Callback', @onPlatformZero);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'STATUS', ...
    'TooltipString', '현재 플랫폼/상차 상태를 로그에 출력합니다.', ...
    'Position', [418 y 72 28], 'Callback', @onShowLoadingState);
y = y - 34;
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'Z-10', ...
    'TooltipString', '플랫폼 리프트를 현재 층 기준 -10 mm 미세 이동합니다.', ...
    'Position', [18 y 43 28], 'Callback', @(~,~)onPlatformJog(-10));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'Z-5', ...
    'TooltipString', '플랫폼 리프트를 현재 층 기준 -5 mm 미세 이동합니다.', ...
    'Position', [65 y 43 28], 'Callback', @(~,~)onPlatformJog(-5));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'Z-1', ...
    'TooltipString', '플랫폼 리프트를 현재 층 기준 -1 mm 미세 이동합니다.', ...
    'Position', [112 y 43 28], 'Callback', @(~,~)onPlatformJog(-1));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'Z+1', ...
    'TooltipString', '플랫폼 리프트를 현재 층 기준 +1 mm 미세 이동합니다.', ...
    'Position', [159 y 43 28], 'Callback', @(~,~)onPlatformJog(1));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'Z+5', ...
    'TooltipString', '플랫폼 리프트를 현재 층 기준 +5 mm 미세 이동합니다.', ...
    'Position', [206 y 43 28], 'Callback', @(~,~)onPlatformJog(5));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'Z+10', ...
    'TooltipString', '플랫폼 리프트를 현재 층 기준 +10 mm 미세 이동합니다.', ...
    'Position', [253 y 50 28], 'Callback', @(~,~)onPlatformJog(10));
y = y - 34;
uicontrol(ctrl, 'Style', 'text', 'String', 'Platform / Pusher Manual', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 220 22]);
y = y - 34;
uicontrol(ctrl, 'Style', 'text', 'String', 'P Abs', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y+5 42 20]);
h.pusherMmEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '260', ...
    'TooltipString', '푸셔 절대 위치 mm', ...
    'Position', [62 y 50 28]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'ABS', ...
    'TooltipString', '입력한 절대 mm 위치로 푸셔를 이동합니다.', ...
    'Position', [118 y 42 28], 'Callback', @onPusherMoveCustom);
uicontrol(ctrl, 'Style', 'text', 'String', 'Jog', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [170 y+5 34 20]);
h.pusherJogEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '10', ...
    'TooltipString', '푸셔 상대 이동량 mm', ...
    'Position', [206 y 44 28]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'P-', ...
    'TooltipString', '푸셔를 입력한 mm만큼 역방향 상대 이동합니다.', ...
    'Position', [256 y 42 28], 'Callback', @(~,~)onPusherJogCustom(-1));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'P+', ...
    'TooltipString', '푸셔를 입력한 mm만큼 정방향 상대 이동합니다.', ...
    'Position', [304 y 42 28], 'Callback', @(~,~)onPusherJogCustom(1));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'P HOME', ...
    'TooltipString', '현재 푸셔 위치를 0으로 설정합니다.', ...
    'Position', [352 y 68 28], 'Callback', @onPusherHome);
y = y - 34;
uicontrol(ctrl, 'Style', 'text', 'String', 'Barrier F1/F2', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y+5 74 20]);
uicontrol(ctrl, 'Style', 'text', 'String', 'UP', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [94 y+5 24 20]);
h.barrierUpEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '90', ...
    'TooltipString', '차단봉 열림 각도 deg', ...
    'Position', [120 y 42 28]);
uicontrol(ctrl, 'Style', 'text', 'String', 'DN', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [168 y+5 24 20]);
h.barrierDownEdit = uicontrol(ctrl, 'Style', 'edit', 'String', '10', ...
    'TooltipString', '차단봉 닫힘 각도 deg', ...
    'Position', [194 y 42 28]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'SET ALL', ...
    'TooltipString', '입력한 차단봉 열림/닫힘 각도를 1,2층 모두에 런타임 적용합니다.', ...
    'Position', [242 y 62 28], 'Callback', @onBarrierTuning);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F1 UP', ...
    'TooltipString', '1층 차단봉을 현재 열림 각도로 올립니다.', ...
    'Position', [310 y 54 28], 'Callback', @(~,~)onBarrierState("up", 1));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F1 DN', ...
    'TooltipString', '1층 차단봉을 현재 닫힘 각도로 내립니다.', ...
    'Position', [370 y 54 28], 'Callback', @(~,~)onBarrierState("down", 1));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F2 UP', ...
    'TooltipString', '2층 차단봉을 현재 열림 각도로 올립니다.', ...
    'Position', [430 y 54 28], 'Callback', @(~,~)onBarrierState("up", 2));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'F2 DN', ...
    'TooltipString', '2층 차단봉을 현재 닫힘 각도로 내립니다.', ...
    'Position', [490 y 54 28], 'Callback', @(~,~)onBarrierState("down", 2));
y = y - 34;
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'TILT OUT', ...
    'Position', [18 y 70 28], 'Callback', @(~,~)onPlatformTilt(18));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'TILT 0', ...
    'Position', [94 y 56 28], 'Callback', @(~,~)onPlatformTilt(0));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'PUSH', ...
    'Position', [156 y 58 28], 'Callback', @(~,~)onPusherMove("main", 260));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'SIDE', ...
    'Position', [220 y 56 28], 'Callback', @(~,~)onPusherMove("side", 120));
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'UNLOAD', ...
    'Position', [282 y 68 28], 'Callback', @onUnloadEstimate);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'U OK', ...
    'Position', [356 y 60 28], 'Callback', @onUnloadConfirm);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'U WAIT', ...
    'TooltipString', '선택된 층의 하차 대기 높이로 플랫폼을 이동한 뒤 하차 방향으로 기울입니다.', ...
    'Position', [422 y 72 28], 'Callback', @onUnloadWaitPose);

y = y - 38;
uicontrol(ctrl, 'Style', 'text', 'String', 'Recent Logs', ...
    'FontWeight', 'bold', 'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', 'Position', [18 y 150 22]);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'COPY LOG', ...
    'Position', [300 y 90 24], 'Callback', @onCopyLog);
uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'OPEN LOG', ...
    'Position', [398 y 100 24], 'Callback', @onOpenLog);
y = y - 190;
h.logBox = uicontrol(ctrl, 'Style', 'listbox', 'String', {'-'}, ...
    'FontName', 'Monospaced', 'FontSize', 9, ...
    'Position', [18 y 480 185]);

tmr = timer('ExecutionMode', 'fixedSpacing', ...
    'BusyMode', 'drop', ...
    'Period', 0.05, ...
    'TimerFcn', @onTick);
start(tmr);
if ~sameText(state.initialTwinOpenError, "")
    appendLog("Initial twin view open failed: " + state.initialTwinOpenError);
end
appendLog("Panel ready. Press CONNECT ROS2; twin view opens immediately and follows ROS DB/status.");

    function ensureLiveTwinVisible(reason)
        if nargin < 1
            reason = "";
        end
        try
            rows = state.displayRows;
            if isempty(rows)
                rows = state.dbRows;
            end
            if isempty(rows) && ~hasPlatformParcel()
                rows = struct([]);
            end
            open_refuge_live_twin_view(rows, liveStatus(), state.lastPlanText, true);
            if ~sameText(reason, "")
                appendLog("Twin view ready: " + string(reason));
            end
        catch ME
            appendLog("Twin view open failed: " + string(ME.message));
        end
    end

    function tf = liveTwinFigureOpen()
        try
            tf = ~isempty(findall(0, 'Type', 'figure', 'Name', 'Refuge Live Digital Twin'));
        catch
            tf = false;
        end
    end

    function onConnect(~, ~)
        try
            if state.rosReady
                updateRosLabel();
                state.followActualDb = true;
                state.renderedDbSeq = -1;
                state.lastRenderedSignature = "";
                ensureLiveTwinVisible("ROS2 already connected");
                return;
            end
            nodeName = sprintf('refuge_matlab_panel_%04d', randi(9999));
            state.node = ros2node(nodeName);
            state.cmdPub = makePublisher(state.node, "/refuge/control_cmd");
            state.twinPub = makePublisher(state.node, "/refuge/twin_cmd");
            state.loadingPub = makePublisher(state.node, "/platform/loading_cmd");
            state.statusSub = makeSubscriber(state.node, "/refuge/status", @onStatusMsg);
            state.dbSub = makeSubscriber(state.node, "/refuge/db", @onDbMsg);
            state.twinSub = makeSubscriber(state.node, "/refuge/twin_state", @onTwinMsg);
            state.logSub = makeSubscriber(state.node, "/refuge/log", @onLogMsg);
            state.motionSub = makeSubscriber(state.node, "/refuge/motion_event", @onMotionMsg);
            for floorNo = 1:3
                f = floorNo;
                prefix = string(sprintf("/refuge/floor%d", f));
                state.floorStatusSub{f} = makeSubscriber(state.node, prefix + "/status", @(varargin)onStatusMsgForFloor(f, varargin{:}));
                state.floorDbSub{f} = makeSubscriber(state.node, prefix + "/db", @(varargin)onDbMsgForFloor(f, varargin{:}));
                state.floorTwinSub{f} = makeSubscriber(state.node, prefix + "/twin_state", @(varargin)onTwinMsgForFloor(f, varargin{:}));
                state.floorLogSub{f} = makeSubscriber(state.node, prefix + "/log", @(varargin)onLogMsg(varargin{:}));
                state.floorMotionSub{f} = makeSubscriber(state.node, prefix + "/motion_event", @(varargin)onMotionMsg(varargin{:}));
                state.floorCmdPub{f} = makePublisher(state.node, prefix + "/control_cmd");
                state.floorTwinPub{f} = makePublisher(state.node, prefix + "/twin_cmd");
            end
            state.loadingStateSub = makeSubscriber(state.node, "/platform/loading_state", @onLoadingStateMsg);
            state.loadingEventSub = makeSubscriber(state.node, "/platform/loading_events", @onLoadingEventMsg);
            state.rosReady = true;
            appendLog("ROS2 connected: " + string(nodeName));
            state.followActualDb = true;
            state.renderedDbSeq = -1;
            state.lastRenderedSignature = "";
            state.lastStatusRenderSignature = "";
            state.lastPlanText = "ROS2 connected | waiting for DB/status | planner ready";
            updatePlanLabel();
            ensureLiveTwinVisible("ROS2 connected");
        catch ME
            state.rosReady = false;
            appendLog("ROS2 connect failed: " + string(ME.message));
        end
        updateRosLabel();
    end

    function onFixedDb(~, ~)
        if state.floorId == 2
            sendControl(struct('cmd', 'test_db_floor2', 'floor', 2));
            state.lastPlanText = 'FIXED DB requested: F2 2314323131243423';
        else
            sendControl(struct('cmd', 'test_db', 'floor', state.floorId));
            state.lastPlanText = sprintf('FIXED DB requested: F%d 231313221132113322', state.floorId);
        end
        updatePlanLabel();
    end

    function onLoadStart(~, ~)
        parcelType = round(str2double(get(h.loadTypeEdit, 'String')));
        if isnan(parcelType) || parcelType < 1 || parcelType > 4
            appendLog("Load Type must be 1..4");
            return;
        end
        sendTwin(struct('cmd', 'load_start', 'type', parcelType, 'target_floor', state.floorId));
        state.lastPlanText = sprintf('MANUAL LOAD requested: type %d to F%d at B4 top', parcelType, state.floorId);
        updatePlanLabel();
    end

    function onLoadComplete(~, ~)
        sendTwin(struct('cmd', 'load_complete'));
        state.lastPlanText = 'LOAD COMPLETE requested: sync current layout for circulation/unload';
        updatePlanLabel();
    end

    function onLoadMode(~, ~)
        state.operatorMode = "load";
        sendLoading(struct('cmd', 'set_mode', 'mode', 'load', 'target_floor', state.floorId));
        state.lastPlanText = sprintf('LOAD MODE: F1/F2 barriers up; selected target F%d, use F1/F2 LOAD to change', state.floorId);
        updatePlanLabel();
    end

    function onUnloadMode(~, ~)
        state.operatorMode = "unload";
        sendLoading(struct('cmd', 'set_mode', 'mode', 'unload'));
        state.lastPlanText = 'UNLOAD MODE: select target P id, then unload starts automatically';
        updatePlanLabel();
    end

    function onIdleMode(~, ~)
        state.operatorMode = "idle";
        sendLoading(struct('cmd', 'set_mode', 'mode', 'idle'));
        sendTwin(struct('cmd', 'sim_auto_stop'));
        state.lastPlanText = 'IDLE MODE: automatic load/unload disabled';
        updatePlanLabel();
    end

    function onCameraLoadStart(~, ~)
        sendLoading(struct('cmd', 'start', 'target_floor', state.floorId));
        state.lastPlanText = sprintf('CAMERA LOAD requested: platform to F%d then B4 load', state.floorId);
        updatePlanLabel();
    end

    function onLoadTargetFloor(floorNo)
        state.floorId = floorNo;
        sendLoading(struct('cmd', 'set_floor', 'floor', floorNo));
        state.lastPlanText = sprintf('LOAD TARGET selected: F%d', floorNo);
        updateLoadTargetLabel();
        updatePlanLabel();
    end

    function onCameraLoadStop(~, ~)
        sendLoading(struct('cmd', 'stop'));
        sendLoading(struct('cmd', 'set_mode', 'mode', 'idle'));
        state.operatorMode = "idle";
        state.lastPlanText = 'CAMERA LOAD stop requested';
        updatePlanLabel();
    end

    function onPusherTuning(~, ~)
        baseMm = str2double(h.pushContactEdit.String);
        extraMm = str2double(h.pushExtraEdit.String);
        assistMm = str2double(h.pushAssistEdit.String);
        if any(isnan([baseMm extraMm assistMm]))
            appendLog("Pusher tuning values must be numeric");
            return;
        end
        sendLoading(struct('cmd', 'set_pusher_tuning', ...
            'contact_to_b4_mm', baseMm, ...
            'extra_mm', extraMm, ...
            'b4_assist_mm', assistMm));
        state.lastPlanText = sprintf('PUSHER TUNE: base %.1f extra %.1f assist %.1f mm', ...
            baseMm, extraMm, assistMm);
        updatePlanLabel();
    end

    function onShowLoadingState(~, ~)
        appendLog("platform state: " + loadingStateText(state.loadingState));
    end

    function onSyncView(~, ~)
        if isempty(state.dbRows)
            appendLog("No /refuge/db received yet");
            ensureLiveTwinVisible("sync view without DB");
            return;
        end
        syncSimFromRows(state.dbRows, "manual sync");
        ensureLiveTwinVisible("manual sync");
    end

    function onStartAuto(~, ~)
        startUnloadTarget("button");
    end

    function onTargetEdited(~, ~)
        if state.operatorMode == "unload"
            startUnloadTarget("target edit");
        end
    end

    function startUnloadTarget(source)
        target = targetId();
        floorNo = floorForTargetId(target);
        if floorNo ~= state.floorId
            state.floorId = floorNo;
            updateLoadTargetLabel();
        end
        sendTwin(struct('cmd', 'sim_auto', 'target', target, 'floor', floorNo));
        state.lastPlanText = sprintf('UNLOAD requested: P%d F%d (%s)', target, floorNo, char(source));
        updatePlanLabel();
    end

    function onStopAuto(~, ~)
        sendTwin(struct('cmd', 'sim_auto_stop'));
    end

    function onStopAll(~, ~)
        sendTwin(struct('cmd', 'stop_all'));
        sendControl(struct('cmd', 'stop'));
    end

    function onClearFault(~, ~)
        sendControl(struct('cmd', 'clear_fault'));
    end

    function onPlanOnce(~, ~)
        sendTwin(struct('cmd', 'plan', 'target', targetId()));
    end

    function onSetRender(enabled)
        sendTwin(struct('cmd', 'set_render', 'auto_plan_images', double(enabled)));
    end

    function onPlatformFloor(floorNo)
        state.floorId = floorNo;
        sendLoading(struct('cmd', 'set_floor', 'floor', floorNo));
        sendLoading(struct('cmd', 'lift_floor', 'floor', floorNo));
        state.lastPlanText = sprintf('PLATFORM target floor set: F%d', floorNo);
        updateLoadTargetLabel();
        updatePlanLabel();
    end

    function onPlatformHome(~, ~)
        sendLoading(struct('cmd', 'lift_floor', 'floor', state.floorId));
    end

    function onPlatformJog(deltaMm)
        sendLoading(struct('cmd', 'lift_jog', 'floor', state.floorId, 'mm', deltaMm));
        state.lastPlanText = sprintf('PLATFORM F%d fine jog %+g mm', state.floorId, deltaMm);
        updatePlanLabel();
    end

    function onPlatformJogCustom(~, ~)
        deltaMm = str2double(h.platformJogEdit.String);
        if isnan(deltaMm) || abs(deltaMm) < 0.001
            appendLog("Platform jog mm must be a non-zero number");
            return;
        end
        onPlatformJog(deltaMm);
    end

    function onPlatformZero(~, ~)
        sendLoading(struct('cmd', 'lift_zero', 'floor', state.floorId));
        state.lastPlanText = sprintf('PLATFORM F%d current height accepted as zero/reference', state.floorId);
        updatePlanLabel();
    end

    function onPlatformTilt(angleDeg)
        sendLoading(struct('cmd', 'platform_tilt', 'angle_deg', angleDeg));
    end

    function onUnloadWaitPose(~, ~)
        if loadingBusy()
            appendLog("platform busy; skipped unload wait pose");
            return;
        end
        floorNo = state.floorId;
        tiltDeg = 18.0;
        state.operatorMode = "unload";
        sendLoading(struct('cmd', 'set_mode', ...
            'mode', 'unload', ...
            'target_floor', floorNo, ...
            'source', 'matlab_unload_wait_pose'));
        sendLoading(struct('cmd', 'platform_tilt', ...
            'floor', floorNo, ...
            'angle_deg', tiltDeg, ...
            'source', 'matlab_unload_wait_pose'));
        state.lastPlanText = sprintf('UNLOAD WAIT POSE: F%d receive height, tilt %.1f deg', floorNo, tiltDeg);
        updatePlanLabel();
    end

    function tf = loadingBusy()
        loading = state.loadingState;
        tf = false;
        if ~isstruct(loading)
            return;
        end
        tf = logicalNumberField(loading, 'active', false) > 0 || ...
            logicalNumberField(loading, 'lift_active', false) > 0 || ...
            logicalNumberField(loading, 'pusher_main_active', false) > 0;
    end

    function onPusherMove(axisName, mm)
        sendLoading(struct('cmd', 'pusher_move', 'axis', char(axisName), 'mm', mm));
    end

    function onPusherMoveCustom(~, ~)
        targetMm = str2double(h.pusherMmEdit.String);
        if isnan(targetMm) || targetMm < 0
            appendLog("Pusher target mm must be a non-negative number");
            return;
        end
        onPusherMove("main", targetMm);
        state.lastPlanText = sprintf('PUSHER move absolute %.1f mm', targetMm);
        updatePlanLabel();
    end

    function onPusherJog(deltaMm)
        sendLoading(struct('cmd', 'pusher_jog', 'axis', 'main', 'mm', deltaMm));
        state.lastPlanText = sprintf('PUSHER relative jog %+g mm', deltaMm);
        updatePlanLabel();
    end

    function onPusherJogCustom(signValue)
        jogMm = str2double(h.pusherJogEdit.String);
        if isnan(jogMm) || abs(jogMm) < 0.001
            appendLog("Pusher jog mm must be a non-zero number");
            return;
        end
        onPusherJog(signValue * abs(jogMm));
    end

    function onPusherHome(~, ~)
        sendLoading(struct('cmd', 'pusher_home', 'axis', 'main'));
    end

    function onBarrierTuning(~, ~)
        [ok, upAngle, downAngle] = readBarrierAngles();
        if ~ok
            return;
        end
        for floorNo = [1 2]
            sendLoading(struct('cmd', 'set_barrier_tuning', ...
                'floor', floorNo, ...
                'up_angle_deg', barrierUpAngleForFloor(floorNo, upAngle), ...
                'down_angle_deg', downAngle));
        end
        state.lastPlanText = sprintf('BARRIER tune: UP %.1f deg / DOWN %.1f deg', upAngle, downAngle);
        updatePlanLabel();
    end

    function onBarrierState(barrierState, floorNo)
        if nargin < 2
            floorNo = state.selectedFloor;
        end
        [ok, upAngle, downAngle] = readBarrierAngles();
        if ~ok
            return;
        end
        sendLoading(struct('cmd', 'set_barrier_tuning', ...
            'floor', floorNo, ...
            'up_angle_deg', barrierUpAngleForFloor(floorNo, upAngle), ...
            'down_angle_deg', downAngle));
        sendLoading(struct('cmd', 'barrier', 'floor', floorNo, 'state', char(barrierState), 'source', 'matlab_panel'));
        state.lastPlanText = sprintf('F%d BARRIER %s', floorNo, upper(char(barrierState)));
        updatePlanLabel();
    end

    function [ok, upAngle, downAngle] = readBarrierAngles()
        upAngle = str2double(h.barrierUpEdit.String);
        downAngle = str2double(h.barrierDownEdit.String);
        if any(isnan([upAngle downAngle])) || any([upAngle downAngle] < 0) || any([upAngle downAngle] > 180)
            appendLog("Barrier angles must be numeric values from 0 to 180 deg");
            ok = false;
            return;
        end
        ok = true;
    end

    function correctedAngle = barrierUpAngleForFloor(floorNo, requestedAngle)
        correctedAngle = requestedAngle;
        if round(double(floorNo)) == 2 && correctedAngle < 120
            correctedAngle = 120;
        end
    end

    function onUnloadEstimate(~, ~)
        sendControl(struct('cmd', 'unload_estimate', ...
            'id', targetId(), 'floor', state.floorId, 'remove', true));
    end

    function onUnloadConfirm(~, ~)
        sendControl(struct('cmd', 'unload_confirm', ...
            'id', targetId(), 'uncertainty_mm', 5.0));
    end

    function onCopyLog(~, ~)
        try
            clipboard('copy', char(strjoin(state.lastLogLines, newline)));
            appendLog("copied recent logs to clipboard");
        catch ME
            appendLog("copy log failed: " + string(ME.message));
        end
    end

    function onOpenLog(~, ~)
        try
            edit(state.logFile);
        catch ME
            appendLog("open log failed: " + string(ME.message));
        end
    end

    function onFollowChanged(~, ~)
        state.followActualDb = logical(h.followCheck.Value);
    end

    function onLocalFixedDb(~, ~)
        state.localRunning = false;
        state.localMode = true;
        S = seedLocalFixedDb();
        parcel_manual_animation_update(S);
        state.lastPlanText = 'LOCAL fixed DB loaded';
        h.planLabel.String = state.lastPlanText;
        h.statusLabel.String = statusLine(S);
    end

    function onLocalStart(~, ~)
        state.localMode = true;
        S = parcel_manual_core_step("snapshot", 0);
        if S.phase ~= 0
            state.localRunning = true;
            return;
        end
        S = parcel_manual_core_step("unload", targetId());
        parcel_manual_animation_update(S);
        state.localRunning = true;
        state.lastPlanText = sprintf('LOCAL circulation: P%d', targetId());
        h.planLabel.String = state.lastPlanText;
    end

    function onLocalStop(~, ~)
        state.localRunning = false;
        state.lastPlanText = 'LOCAL stopped';
        h.planLabel.String = state.lastPlanText;
    end

    function onLocalReset(~, ~)
        state.localRunning = false;
        state.localMode = true;
        S = parcel_manual_core_step("reset", 0);
        parcel_manual_animation_update(S);
        h.statusLabel.String = 'Local sim reset';
    end

    function onTick(~, ~)
        if ~isvalid(ctrl)
            return;
        end
        if state.localRunning
            steps = speedSteps();
            for k = 1:steps
                S = parcel_manual_core_step("step", 0);
            end
            parcel_manual_animation_update(S);
            h.statusLabel.String = statusLine(S);
            if isfield(S, 'circCompleteTargetId') && S.circCompleteTargetId > 0
                state.localRunning = false;
            end
        elseif state.followActualDb && (~isempty(state.dbRows) || hasPlatformParcel() || liveTwinFigureOpen())
            renderActualDbIfNeeded();
        end
        updateStatusLabels();
        updatePlatformHeightLabel();
        updatePlanLabel();
    end

    function onStatusMsg(varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if isstruct(payload)
            state.status = payload;
            if clearStaleMotionIfIdle()
                state.renderedDbSeq = -1;
                state.lastRenderedSignature = "";
            end
        end
    end

    function onStatusMsgForFloor(floorNo, varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if isstruct(payload)
            payload.floor = floorNo;
            state.floorStatus{floorNo} = payload;
            if floorNo == state.floorId || statusLooksActive(payload)
                state.status = payload;
                if clearStaleMotionIfIdle()
                    state.renderedDbSeq = -1;
                    state.lastRenderedSignature = "";
                end
            end
        end
    end

    function onDbMsg(varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if isstruct(payload)
            rows = payload(:);
            sig = dbSignature(rows);
            state.dbRows = rows;
            if ~sameText(sig, state.lastDbSignature)
                state.dbSeq = state.dbSeq + 1;
                state.lastDbSignature = sig;
            end
        else
            state.dbRows = struct([]);
        end
    end

    function onDbMsgForFloor(floorNo, varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if isstruct(payload)
            state.floorDbRows{floorNo} = addFloorToRows(payload(:), floorNo);
        else
            state.floorDbRows{floorNo} = struct([]);
        end
        mergeFloorDbRows();
    end

    function onTwinMsg(varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if isstruct(payload)
            state.twin = payload;
        end
    end

    function onTwinMsgForFloor(floorNo, varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if isstruct(payload)
            payload.floor = floorNo;
            state.floorTwin{floorNo} = payload;
            if floorNo == state.floorId || statusLooksActive(payload)
                state.twin = payload;
            end
        end
    end

    function mergeFloorDbRows()
        parts = cell(1,3);
        hasRows = false;
        for ff = 1:3
            rows = state.floorDbRows{ff};
            if isstruct(rows) && ~isempty(rows)
                parts{ff} = rows(:);
                hasRows = true;
            else
                parts{ff} = struct([]);
            end
        end
        if ~hasRows
            return;
        end
        rows = concatStructRows(parts);
        sig = dbSignature(rows);
        state.dbRows = rows;
        if ~sameText(sig, state.lastDbSignature)
            state.dbSeq = state.dbSeq + 1;
            state.lastDbSignature = sig;
        end
    end


    function onLogMsg(varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if isstruct(payload)
            ev = textField(payload, 'event', 'log');
            appendLog(string(ev) + " " + compactStructText(payload));
            if isMotionEventName(ev)
                updateMotionFromEvent(ev, payload, false);
            elseif isDbRefreshEventName(ev)
                refreshDisplayFromLatestDb(string(ev));
            elseif posixNow() - state.motionEventLastAt > 2.0
                updateMotionFromEvent(ev, payload, false);
            end
        else
            appendLog(string(messageText(msg)));
        end
    end

    function onMotionMsg(varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if ~isstruct(payload)
            return;
        end
        state.motionEventLastAt = posixNow();
        ev = textField(payload, 'event', 'motion');
        updateMotionFromEvent(ev, payload, true);
    end

    function onLoadingStateMsg(varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if ~isstruct(payload)
            return;
        end
        state.loadingState = payload;
        updatePlatformHeightLabel();
        updateLoadTargetLabel();
        sig = loadingStateSignature(payload);
        if ~sameText(sig, state.lastLoadingStateSignature)
            state.lastLoadingStateSignature = sig;
            state.lastStatusRenderSignature = "";
            appendLog("platform state: " + loadingStateText(payload));
        end
    end

    function onLoadingEventMsg(varargin)
        msg = varargin{end};
        payload = decodeJsonMessage(msg);
        if isstruct(payload)
            ev = textField(payload, 'event', 'platform_event');
            appendLog("platform " + string(ev) + " " + compactStructText(payload));
        else
            appendLog("platform " + string(messageText(msg)));
        end
    end

    function renderActualDbIfNeeded()
        if state.motion.active
            if clearStaleMotionIfIdle()
                state.renderedDbSeq = -1;
                state.lastRenderedSignature = "";
            else
                renderMotionTimeline(false);
                return;
            end
        end
        if state.motion.active
            renderMotionTimeline(false);
            return;
        end
        movingRows = activeMovePreviewRows();
        if ~isempty(movingRows)
            sig = dbSignature(movingRows) + "|" + state.livePreviewKey;
            if ~sameText(sig, state.lastRenderedSignature)
                syncSimFromRows(movingRows, "live motion preview", true);
                state.lastRenderedSignature = sig;
            end
            return;
        end
        if state.dbSeq ~= state.renderedDbSeq
            sig = dbSignature(state.dbRows);
            syncSimFromRows(state.dbRows, "db update");
            state.lastRenderedSignature = sig;
            state.renderedDbSeq = state.dbSeq;
            state.displayRows = state.dbRows;
            state.lastStatusRenderSignature = statusRenderSignature(liveStatus());
            return;
        end
        if isempty(state.displayRows) && isempty(state.dbRows) && hasPlatformParcel()
            statusSig = statusRenderSignature(liveStatus());
            if ~sameText(statusSig, state.lastStatusRenderSignature)
                open_refuge_live_twin_view(struct([]), liveStatus(), "platform detection", false);
                state.lastStatusRenderSignature = statusSig;
            end
            return;
        end
        if isempty(state.displayRows) && isempty(state.dbRows) && liveTwinFigureOpen()
            statusSig = statusRenderSignature(liveStatus());
            if ~sameText(statusSig, state.lastStatusRenderSignature)
                open_refuge_live_twin_view(struct([]), liveStatus(), state.lastPlanText, false);
                state.lastStatusRenderSignature = statusSig;
            end
            return;
        end
        statusSig = statusRenderSignature(liveStatus());
        if ~sameText(statusSig, "") && ~sameText(statusSig, state.lastStatusRenderSignature)
            syncSimFromRows(state.displayRows, "state update");
            state.lastStatusRenderSignature = statusSig;
        end
    end

    function updateMotionFromEvent(ev, payload, fromMotionTopic)
        if nargin < 3
            fromMotionTopic = false;
        end
        ev = string(ev);
        if ev == "move_cmd"
            state.lastMoveCmd = payload;
            state.lastPlanText = sprintf('CMD: B%d %s %.1f mm | %s', ...
                round(numberField(payload, 'belt', 0)), ...
                dirText(numberField(payload, 'dir', 0)), ...
                numberField(payload, 'mm', 0), ...
                textField(payload, 'reason', 'move'));
            h.planLabel.String = state.lastPlanText;
        elseif ev == "move_start"
            startMotionFromEvent(payload, fromMotionTopic);
        elseif ev == "move_done"
            finishMotionFromEvent(payload);
        elseif ev == "stop" || ev == "stop_all" || ev == "fault"
            state.motion = emptyMotion();
            state.renderedDbSeq = -1;
            state.lastRenderedSignature = "";
        end
    end

    function startMotionFromEvent(payload, fromMotionTopic)
        if nargin < 2
            fromMotionTopic = false;
        end
        belt1 = round(numberField(payload, 'belt', 0));
        if belt1 <= 0
            return;
        end
        cmd = state.lastMoveCmd;
        if fromMotionTopic
            cmd = mergeStructs(cmd, payload);
        end
        if ~sameMoveCommand(cmd, payload)
            cmd = struct();
        end
        baseRows = state.dbRows;
        if isempty(baseRows)
            baseRows = state.displayRows;
        end
        if isempty(baseRows)
            return;
        end
        targetMm = numberField(payload, {'target_mm','mm'}, NaN);
        if isnan(targetMm) || targetMm <= 0
            targetMm = numberField(cmd, {'target_mm','mm'}, 0);
        end
        if targetMm <= 0
            return;
        end
        state.motion = emptyMotion();
        state.motion.active = true;
        state.motion.startedAt = posixNow();
        state.motion.belt = belt1 - 1;
        state.motion.dir = sign(numberField(payload, 'dir', numberField(cmd, 'dir', 0)));
        state.motion.targetMm = targetMm;
        state.motion.commandMm = numberField(cmd, 'mm', numberField(payload, 'mm', targetMm));
        state.motion.rpm = max(1.0, numberField(cmd, 'rpm', motionDefaultRpm()));
        state.motion.reason = char(textField(cmd, 'reason', 'move'));
        state.motion.handoffId = round(numberField(cmd, 'handoff_id', 0));
        state.motion.handoffReceiver = round(numberField(cmd, 'handoff_receiver', 0)) - 1;
        state.motion.baseRows = baseRows;
        state.motion.lastProgress = -999.0;
        state.motion.key = sprintf('%d:%d:%.2f:%.3f', state.motion.belt, state.motion.dir, targetMm, state.motion.startedAt);
        renderMotionTimeline(true);
    end

    function finishMotionFromEvent(payload)
        if ~state.motion.active
            refreshDisplayFromLatestDb("move_done_without_active_motion");
            return;
        end
        belt1 = round(numberField(payload, 'belt', 0));
        direction = sign(numberField(payload, 'dir', state.motion.dir));
        if belt1 > 0 && (belt1 - 1) ~= state.motion.belt
            state.motion = emptyMotion();
            refreshDisplayFromLatestDb("move_done_belt_mismatch");
            return;
        end
        if direction ~= 0 && direction ~= state.motion.dir
            state.motion = emptyMotion();
            refreshDisplayFromLatestDb("move_done_dir_mismatch");
            return;
        end
        traveled = numberField(payload, {'traveled_mm','target_mm','requested_mm'}, state.motion.targetMm);
        rows = rowsAfterBeltMove(state.motion.baseRows, state.motion.belt, state.motion.dir, traveled, state.motion.reason, state.motion.handoffId, state.motion.handoffReceiver);
        state.displayRows = rows;
        state.motion = emptyMotion();
        syncSimFromRows(rows, "move done", true);
        state.lastRenderedSignature = dbSignature(rows) + "|move_done";
        state.renderedDbSeq = -1;
    end

    function renderMotionTimeline(forceRender)
        if ~state.motion.active
            return;
        end
        elapsed = max(0.0, posixNow() - state.motion.startedAt);
        speed = motionSpeedMmSec(state.motion.reason, state.motion.rpm);
        encoderProgress = encoderProgressFromStatus(state.motion.belt, state.motion.dir, state.motion.targetMm);
        if ~isnan(encoderProgress)
            progress = min(state.motion.targetMm, max(0.0, encoderProgress));
        else
            progress = min(state.motion.targetMm, elapsed * speed);
        end
        if ~forceRender && abs(progress - state.motion.lastProgress) < 0.35 && progress < state.motion.targetMm
            return;
        end
        state.motion.lastProgress = progress;
        rows = rowsAfterBeltMove(state.motion.baseRows, state.motion.belt, state.motion.dir, progress, state.motion.reason, state.motion.handoffId, state.motion.handoffReceiver);
        state.displayRows = rows;
        state.livePreviewKey = sprintf('%s:%.2f', state.motion.key, progress);
        syncSimFromRows(rows, "motion timeline", true);
        state.lastRenderedSignature = dbSignature(rows) + "|" + state.livePreviewKey;
    end

    function rpm = motionDefaultRpm()
        rpm = 30.0;
        if isstruct(state.status) && isfield(state.status, 'motion_tuning') && isstruct(state.status.motion_tuning)
            rpm = numberField(state.status.motion_tuning, 'default_rpm', rpm);
        end
    end

    function rows = activeMovePreviewRows()
        rows = struct([]);
        if isempty(state.dbRows) || ~isstruct(state.status) || ~isfield(state.status, 'pending_move')
            state.lastMoveKey = "";
            return;
        end
        pending = state.status.pending_move;
        if isempty(pending) || ~isstruct(pending)
            state.lastMoveKey = "";
            return;
        end
        started = logical(numberField(pending, 'started', 0));
        if ~started
            state.livePreviewKey = "";
            return;
        end
        belt = round(numberField(pending, 'belt', -1));
        direction = sign(numberField(pending, 'dir', 0));
        targetMm = numberField(pending, {'target_mm','mm'}, 0);
        rpm = max(1, numberField(pending, 'rpm', 30));
        startedAt = numberField(pending, 'started_at', NaN);
        if belt < 0 || belt > 3 || direction == 0 || targetMm <= 0 || isnan(startedAt)
            state.livePreviewKey = "";
            return;
        end
        if ~pendingMoveIsFresh(pending)
            state.lastMoveKey = "";
            state.livePreviewKey = "";
            return;
        end
        moveKey = sprintf('%d:%d:%.3f:%.3f', belt, direction, targetMm, startedAt);
        if ~sameText(moveKey, state.lastMoveKey)
            state.lastMoveKey = moveKey;
            state.moveBaseRows = state.dbRows;
            state.lastMoveRenderMm = -999;
        end
        elapsed = max(0, posixNow() - startedAt);
        beltSpeedMmSec = max(12.0, rpm * 1.6);
        encoderProgress = encoderProgressFromStatus(belt, direction, targetMm);
        if ~isnan(encoderProgress)
            progress = min(targetMm, max(0.0, encoderProgress));
        else
            progress = min(targetMm, elapsed * beltSpeedMmSec);
        end
        if abs(progress - state.lastMoveRenderMm) < 0.75 && progress < targetMm
            return;
        end
        state.lastMoveRenderMm = progress;
        state.livePreviewKey = sprintf('%s:%.2f', moveKey, progress);
        rows = state.moveBaseRows;
        handoffId = round(numberField(pending, 'handoff_id', 0));
        handoffReceiver = round(numberField(pending, 'handoff_receiver', 0)) - 1;
        for ii = 1:numel(rows)
            rowBelt = round(numberField(rows(ii), 'belt', -99));
            if rowBelt == belt
                id = round(numberField(rows(ii), 'id', 0));
                startPos = numberField(rows(ii), 'pos', 0);
                if direction > 0 && handoffId > 0 && id == handoffId && handoffReceiver >= 0 && handoffReceiver <= 3
                    sourceAxis = axisLengthForRow(rows(ii), belt);
                    frontAfter = startPos + sourceAxis / 2.0 + progress;
                    tailAfter = startPos - sourceAxis / 2.0 + progress;
                    sourceLen = beltLengthMm(belt);
                    if tailAfter > sourceLen
                        excess = frontAfter - sourceLen;
                        rows(ii).belt = handoffReceiver;
                        receiverAxis = axisLengthForRow(rows(ii), handoffReceiver);
                        rows(ii).pos = max(receiverAxis / 2.0, 250.0 - receiverAxis / 2.0) + excess;
                    else
                        rows(ii).pos = startPos + progress;
                    end
                else
                    rows(ii).pos = startPos + direction * progress;
                end
            end
        end
    end

    function cleared = clearStaleMotionIfIdle()
        cleared = false;
        if ~state.motion.active
            return;
        end
        moving = numberField(state.status, 'hardware_moving', 0);
        pending = struct();
        pendingActive = false;
        if isstruct(state.status) && isfield(state.status, 'pending_move') && isstruct(state.status.pending_move)
            pending = state.status.pending_move;
            pendingActive = logical(numberField(pending, 'started', 0));
        end
        pendingMatches = false;
        if pendingActive
            pendingBelt = round(numberField(pending, 'belt', -99));
            pendingDir = sign(numberField(pending, 'dir', 0));
            pendingMatches = pendingBelt == state.motion.belt && ...
                (pendingDir == 0 || pendingDir == state.motion.dir);
        end

        encoderProgress = encoderProgressFromStatus(state.motion.belt, state.motion.dir, state.motion.targetMm);
        targetDone = ~isnan(encoderProgress) && ...
            encoderProgress >= max(0.0, state.motion.targetMm - 1.0);
        elapsed = posixNow() - state.motion.startedAt;
        if elapsed < 0.30
            return;
        end
        timeoutSec = max(4.0, state.motion.targetMm / max(8.0, motionSpeedMmSec(state.motion.reason, state.motion.rpm)) + 2.0);

        if moving > 0 && pendingActive && pendingMatches && ~targetDone && elapsed <= timeoutSec
            return;
        end

        state.motion = emptyMotion();
        state.livePreviewKey = "";
        state.lastMoveKey = "";
        if ~isempty(state.dbRows)
            state.displayRows = state.dbRows;
        end
        cleared = true;
    end

    function refreshDisplayFromLatestDb(reason)
        if isempty(state.dbRows)
            state.renderedDbSeq = -1;
            state.lastRenderedSignature = "";
            state.livePreviewKey = "";
            state.lastMoveKey = "";
            return;
        end
        state.motion = emptyMotion();
        state.displayRows = state.dbRows;
        state.livePreviewKey = "";
        state.lastMoveKey = "";
        state.renderedDbSeq = state.dbSeq;
        state.lastRenderedSignature = dbSignature(state.dbRows) + "|" + string(reason);
        syncSimFromRows(state.dbRows, string(reason), true);
    end

    function fresh = pendingMoveIsFresh(pending)
        fresh = false;
        if ~isstruct(pending) || isempty(fieldnames(pending))
            return;
        end
        if numberField(state.status, 'hardware_moving', 0) > 0
            fresh = true;
            return;
        end
        startedAt = numberField(pending, {'started_at','issued_at'}, NaN);
        if isnan(startedAt)
            fresh = true;
            return;
        end
        targetMm = max(0.0, numberField(pending, {'target_mm','mm'}, 0));
        rpm = max(1.0, numberField(pending, 'rpm', motionDefaultRpm()));
        reason = textField(pending, 'reason', 'move');
        timeoutSec = max(4.0, targetMm / max(8.0, motionSpeedMmSec(reason, rpm)) + 3.0);
        fresh = (posixNow() - startedAt) <= timeoutSec;
    end

    function progress = encoderProgressFromStatus(belt0, direction, targetMm)
        progress = NaN;
        if ~isstruct(state.status) || ~isfield(state.status, 'pending_move') || ~isstruct(state.status.pending_move)
            return;
        end
        pending = state.status.pending_move;
        pendingBelt = round(numberField(pending, 'belt', -99));
        pendingDir = sign(numberField(pending, 'dir', 0));
        if pendingBelt ~= round(belt0)
            return;
        end
        if pendingDir ~= 0 && direction ~= 0 && pendingDir ~= sign(direction)
            return;
        end
        progress = numberField(pending, 'encoder_progress_clamped_mm', NaN);
        if isnan(progress)
            progress = numberField(pending, 'encoder_progress_mm', NaN);
        end
        if isnan(progress)
            return;
        end
        progress = max(0.0, progress);
        if nargin >= 3 && targetMm > 0
            progress = min(progress, max(0.0, targetMm));
        end
    end

    function syncSimFromRows(rows, reason, ~)
        if isempty(rows)
            return;
        end
        rows = sortDbRows(rows);
        state.displayRows = rows;
        planText = state.lastPlanText;
        if reason ~= ""
            planText = string(reason) + " | " + string(planText);
        end
        open_refuge_live_twin_view(rows, liveStatus(), planText, false);
        if reason ~= ""
            h.statusLabel.String = sprintf('View synced: %s | %d boxes', reason, numel(rows));
        end
    end

    function merged = liveStatus()
        if isstruct(state.status) && ~isempty(fieldnames(state.status))
            merged = state.status;
        else
            merged = struct();
        end
        loading = state.loadingState;
        if ~isstruct(loading) || isempty(fieldnames(loading))
            return;
        end
        floorNo = round(numberField(loading, 'target_floor', state.floorId));
        floorNo = max(1, min(3, floorNo));
        det = structField(loading, 'latest_detection');
        presentRaw = logicalNumberField(det, 'present', false) > 0;
        present = presentRaw && platformParcelAcceptDetection(loading);

        platform = structField(merged, 'platform');
        platform.floor = floorNo;
        platform.target_floor = floorNo;
        platform.z_mm = numberField(loading, 'current_floor_z_mm', numberField(platform, 'z_mm', 0));
        platform.target_z_mm = numberField(loading, 'target_floor_z_mm', numberField(platform, 'target_z_mm', platform.z_mm));
        platform.visible = logicalNumberField(loading, {'active','lift_active'}, false) > 0 || present || floorNo ~= 1;
        platform.busy = logicalNumberField(loading, 'lift_active', false) > 0;
        platform.confidence = 'platform_loading_state';
        merged.platform = platform;

        pusher = structField(merged, 'pusher');
        pusher.main_mm = numberField(loading, 'pusher_main_mm', numberField(pusher, {'main_mm','pusher_mm','main'}, 0));
        pusher.main_target_mm = numberField(loading, 'pusher_main_target_mm', pusher.main_mm);
        pusher.main_active = logicalNumberField(loading, 'pusher_main_active', false) > 0;
        pusher.confidence = 'platform_loading_state';
        merged.pusher = pusher;

        holdParcel = platformParcelShouldHold(loading);
        if present
            if ~holdParcel && max(pusher.main_mm, pusher.main_target_mm) < 5.0
                state.platformParcelPusherMm = 0.0;
                state.platformParcelSignature = "";
            end
            parcelType = round(numberField(det, 'parcel_type', 0));
            longMm = numberField(det, 'long_mm', 0);
            shortMm = numberField(det, 'short_mm', 0);
            if longMm > 0 && shortMm > 0 && parcelType >= 1 && parcelType <= 4
                parcelSig = sprintf('%d:%d:%.1f:%.1f', floorNo, parcelType, longMm, shortMm);
                if ~sameText(parcelSig, state.platformParcelSignature)
                    state.platformParcelPusherMm = 0.0;
                    state.platformParcelSignature = string(parcelSig);
                end
                parcel = struct();
                parcel.visible = true;
                parcel.floor = floorNo;
                parcel.id = 0;
                parcel.type = parcelType;
                parcel.box_type = parcelType;
                parcel.long_side = longMm;
                parcel.short_side = shortMm;
                parcel.height = 75.0;
                state.platformParcelPusherMm = max([state.platformParcelPusherMm, pusher.main_mm, pusher.main_target_mm]);
                parcel.pusher_mm = state.platformParcelPusherMm;
                parcel.yaw_deg = numberField(det, 'yaw_error_deg', 0);
                parcel.source = 'platform_camera';
                state.lastPlatformParcel = parcel;
                merged.platform_parcel = parcel;
            end
        elseif holdParcel && isstruct(state.lastPlatformParcel) && ~isempty(fieldnames(state.lastPlatformParcel))
            parcel = state.lastPlatformParcel;
            parcel.floor = floorNo;
            parcel.visible = true;
            state.platformParcelPusherMm = max([state.platformParcelPusherMm, pusher.main_mm, pusher.main_target_mm]);
            parcel.pusher_mm = state.platformParcelPusherMm;
            merged.platform_parcel = parcel;
        else
            if isfield(merged, 'platform_parcel')
                merged = rmfield(merged, 'platform_parcel');
            end
            state.lastPlatformParcel = struct();
            state.platformParcelSignature = "";
            state.platformParcelPusherMm = 0.0;
        end
    end

    function tf = hasPlatformParcel()
        loading = state.loadingState;
        if ~isstruct(loading) || isempty(fieldnames(loading))
            tf = false;
            return;
        end
        det = structField(loading, 'latest_detection');
        presentNow = platformParcelAcceptDetection(loading) && ...
            logicalNumberField(det, 'present', false) > 0 && round(numberField(det, 'parcel_type', 0)) >= 1;
        tf = presentNow || (platformParcelShouldHold(loading) && isstruct(state.lastPlatformParcel) && ~isempty(fieldnames(state.lastPlatformParcel)));
    end

    function S = seedLocalFixedDb()
        if state.floorId == 2
            order = [2 3 1 4 3 2 3 1 3 1 2 4 3 4 2 3];
        else
            order = [2 3 1 3 1 3 2 2 1 1 3 2 1 1 3 3 2 2];
        end
        dims = [122 112; 142 102; 162 122; 200 147];
        S = parcel_manual_core_step("reset", 0);
        for ii = 1:numel(order)
            t = order(ii);
            S = parcel_manual_core_step("seq_package", ...
                [state.floorId ii dims(t,1) dims(t,2) 75]);
        end
    end

    function sendControl(payload)
        [pub, routedPayload, routeLabel] = floorPublisherForPayload(payload, state.cmdPub, state.floorCmdPub, "control");
        sendJson(pub, routedPayload, routeLabel);
    end

    function sendTwin(payload)
        [pub, routedPayload, routeLabel] = floorPublisherForPayload(payload, state.twinPub, state.floorTwinPub, "twin");
        sendJson(pub, routedPayload, routeLabel);
    end

    function sendLoading(payload)
        sendJson(state.loadingPub, payload, "platform loading");
    end

    function [pub, routedPayload, routeLabel] = floorPublisherForPayload(payload, fallbackPub, floorPubs, label)
        routedPayload = payload;
        floorNo = floorFromPayload(payload);
        if floorNo >= 1 && floorNo <= numel(floorPubs) && ~isempty(floorPubs{floorNo})
            if ~isfield(routedPayload, 'floor')
                routedPayload.floor = floorNo;
            end
            pub = floorPubs{floorNo};
            routeLabel = sprintf('%s F%d', char(label), floorNo);
            return;
        end
        pub = fallbackPub;
        routeLabel = label;
    end

    function floorNo = floorFromPayload(payload)
        floorNo = state.floorId;
        try
            if isstruct(payload)
                if isfield(payload, 'floor')
                    floorNo = round(double(payload.floor));
                elseif isfield(payload, 'target_floor')
                    floorNo = round(double(payload.target_floor));
                end
            end
        catch
            floorNo = state.floorId;
        end
        if isnan(floorNo) || floorNo < 1
            floorNo = state.floorId;
        end
    end

    function sendJson(pub, payload, label)
        if ~state.rosReady || isempty(pub)
            appendLog("ROS2 is not connected; skipped " + label + " command");
            return;
        end
        try
            text = jsonencode(payload);
            msg = ros2message(pub);
            try
                msg.data = char(text);
            catch
                msg.Data = char(text);
            end
            send(pub, msg);
            appendLog("sent " + label + ": " + string(text));
        catch ME
            appendLog("send failed: " + string(ME.message));
        end
    end

    function updateRosLabel()
        if state.rosReady
            h.rosLabel.String = 'ROS2: connected';
            h.rosLabel.ForegroundColor = [0.05 0.45 0.12];
        else
            h.rosLabel.String = 'ROS2: disconnected';
            h.rosLabel.ForegroundColor = [0.65 0.10 0.10];
        end
    end

    function updateStatusLabels()
        if ~isstruct(state.status) || isempty(fieldnames(state.status))
            if ~state.localRunning
                h.statusLabel.String = 'Status: waiting for /refuge/status';
            end
            return;
        end
        mode = textField(state.status, 'mode', '-');
        target = numberField(state.status, 'target', 0);
        complete = numberField(state.status, 'complete', 0);
        fault = textField(state.status, 'fault', '');
        moving = numberField(state.status, 'hardware_moving', 0);
        h.statusLabel.String = sprintf('Status: %s | target P%d | complete P%d | moving %d | fault %s', ...
            mode, round(target), round(complete), round(moving), fault);
        tof = numericArrayField(state.status, 'tof');
        if ~isempty(tof)
            h.tofLabel.String = sprintf('ToF CH0/2/4/6: %.0f / %.0f / %.0f / %.0f', ...
                safeIndex(tof,1), safeIndex(tof,3), safeIndex(tof,5), safeIndex(tof,7));
        end
    end

    function updatePlatformHeightLabel()
        if ~isfield(h, 'platformHeightLabel') || ~isvalid(h.platformHeightLabel)
            return;
        end
        if ~isstruct(state.loadingState) || isempty(fieldnames(state.loadingState))
            h.platformHeightLabel.String = sprintf('PF F%d | waiting for state', state.floorId);
            return;
        end
        floorNo = round(numberField(state.loadingState, 'target_floor', state.floorId));
        zMm = numberField(state.loadingState, 'current_floor_z_mm', NaN);
        offsetMm = numberField(state.loadingState, 'current_floor_offset_mm', NaN);
        baseMm = numberField(state.loadingState, 'current_floor_base_z_mm', NaN);
        if isnan(zMm)
            h.platformHeightLabel.String = sprintf('PF F%d | state received', floorNo);
        else
            h.platformHeightLabel.String = sprintf('PF F%d | z %.1f mm | offset %+0.1f | base %.1f', ...
                floorNo, zMm, offsetMm, baseMm);
        end
    end

    function updateLoadTargetLabel()
        if ~isfield(h, 'loadTargetLabel') || ~isvalid(h.loadTargetLabel)
            return;
        end
        h.loadTargetLabel.String = sprintf('Selected: F%d', state.floorId);
    end

    function updatePlanLabel()
        txt = planTextFromState();
        if txt == ""
            txt = state.lastPlanText;
        end
        if txt ~= ""
            state.lastPlanText = txt;
            h.planLabel.String = char(txt);
        end
    end

    function txt = planTextFromState()
        txt = "";
        if isstruct(state.status) && isfield(state.status, 'pending_move') && isstruct(state.status.pending_move)
            p = state.status.pending_move;
            belt = numberField(p, 'belt', -1);
            if belt >= 0 && pendingMoveIsFresh(p)
                txt = sprintf('NOW: B%d %s %.1f mm | %s', ...
                    round(belt) + 1, dirText(numberField(p, 'dir', 0)), ...
                    numberField(p, 'mm', 0), textField(p, 'reason', 'move'));
                return;
            end
        end
        if isstruct(state.twin) && isfield(state.twin, 'auto') && isstruct(state.twin.auto)
            auto = state.twin.auto;
            active = numberField(auto, 'active', 0);
            step = numberField(auto, 'step', 0);
            message = textField(auto, 'message', '');
            if active || message ~= ""
                txt = sprintf('AUTO step %.0f | %s', step, message);
            end
        end
        if isstruct(state.twin) && isfield(state.twin, 'plan') && isstruct(state.twin.plan)
            plan = state.twin.plan;
            if isfield(plan, 'moves')
                move = firstStruct(plan.moves);
                if isstruct(move) && ~isempty(fieldnames(move))
                    belt = numberField(move, 'belt', -1);
                    if belt > 0
                        txt = sprintf('NEXT: B%d %s %.1f mm | %s', ...
                            round(belt), dirText(numberField(move, 'dir', 0)), ...
                            numberField(move, 'mm', 0), textField(move, 'message', 'plan'));
                    end
                end
            end
        end
    end

    function appendLog(line)
        line = string(line);
        stamp = string(datetime('now', 'Format', 'HH:mm:ss'));
        state.lastLogLines(end+1,1) = "[" + stamp + "] " + line;
        try
            fid = fopen(state.logFile, 'a');
            if fid > 0
                fprintf(fid, '%s\n', char(state.lastLogLines(end)));
                fclose(fid);
            end
        catch
        end
        if numel(state.lastLogLines) > 80
            state.lastLogLines = state.lastLogLines(end-79:end);
        end
        if isfield(h, 'logBox') && isvalid(h.logBox)
            h.logBox.String = cellstr(state.lastLogLines);
            h.logBox.Value = numel(state.lastLogLines);
        end
    end

    function id = targetId()
        id = round(str2double(h.targetEdit.String));
        if isnan(id) || id < 1
            id = 1;
            h.targetEdit.String = '1';
        end
    end

    function floorNo = floorForTargetId(id)
        floorNo = state.floorId;
        for ff = 1:numel(state.floorDbRows)
            rows = state.floorDbRows{ff};
            if rowsContainId(rows, id)
                floorNo = ff;
                return;
            end
        end
        if rowsContainId(state.dbRows, id)
            for ii = 1:numel(state.dbRows)
                if round(numberField(state.dbRows(ii), 'id', 0)) == id
                    rowFloor = round(numberField(state.dbRows(ii), 'floor', floorNo));
                    if rowFloor >= 1 && rowFloor <= numel(state.floorDbRows)
                        floorNo = rowFloor;
                    end
                    return;
                end
            end
        end
        appendLog(sprintf('target P%d not found in floor DB; using selected F%d', id, floorNo));
    end

    function tf = rowsContainId(rows, id)
        tf = false;
        if ~isstruct(rows) || isempty(rows)
            return;
        end
        for ii = 1:numel(rows)
            if round(numberField(rows(ii), 'id', 0)) == id
                tf = true;
                return;
            end
        end
    end

    function onClose(~, ~)
        try
            if exist('tmr', 'var') && isa(tmr, 'timer') && isvalid(tmr)
                stop(tmr);
                delete(tmr);
            end
        catch
        end
        cleanupRosHandles();
        try
            if isvalid(ctrl)
                delete(ctrl);
            end
        catch
        end
    end

    function cleanupRosHandles()
        fields = ["statusSub","dbSub","twinSub","logSub","motionSub", ...
            "loadingStateSub","loadingEventSub","cmdPub","twinPub","loadingPub", ...
            "node"];
        for ii = 1:numel(fields)
            name = char(fields(ii));
            if isfield(state, name)
                deleteRosHandle(state.(name));
                state.(name) = [];
            end
        end
        for ii = 1:numel(state.floorStatusSub)
            deleteRosHandle(state.floorStatusSub{ii});
            deleteRosHandle(state.floorDbSub{ii});
            deleteRosHandle(state.floorTwinSub{ii});
            deleteRosHandle(state.floorLogSub{ii});
            deleteRosHandle(state.floorMotionSub{ii});
            deleteRosHandle(state.floorCmdPub{ii});
            deleteRosHandle(state.floorTwinPub{ii});
            state.floorStatusSub{ii} = [];
            state.floorDbSub{ii} = [];
            state.floorTwinSub{ii} = [];
            state.floorLogSub{ii} = [];
            state.floorMotionSub{ii} = [];
            state.floorCmdPub{ii} = [];
            state.floorTwinPub{ii} = [];
        end
        state.rosReady = false;
    end

    function deleteRosHandle(obj)
        try
            if ~isempty(obj)
                delete(obj);
            end
        catch
        end
    end
end

function pub = makePublisher(node, topic)
try
    pub = ros2publisher(node, topic, "std_msgs/String");
catch
    pub = ros2publisher(node, topic, "std_msgs/msg/String");
end
end

function sub = makeSubscriber(node, topic, cb)
try
    sub = ros2subscriber(node, topic, "std_msgs/String", cb);
catch
    sub = ros2subscriber(node, topic, "std_msgs/msg/String", cb);
end
end

function yes = isMotionEventName(ev)
ev = string(ev);
yes = any(ev == ["move_cmd","move_start","move_done","stop","stop_all","fault"]);
end

function yes = isDbRefreshEventName(ev)
ev = string(ev);
yes = any(ev == ["db_sync_twin","db_reconcile_sync_sent","tof_box_source_lost_completed","tof_correction_done","compact_done"]);
end

function out = mergeStructs(a, b)
out = struct();
if isstruct(a) && ~isempty(fieldnames(a))
    out = a;
end
if ~isstruct(b)
    return;
end
names = fieldnames(b);
for ii = 1:numel(names)
    out.(names{ii}) = b.(names{ii});
end
end

function rows = addFloorToRows(rows, floorNo)
if ~isstruct(rows) || isempty(rows)
    rows = struct([]);
    return;
end
for ii = 1:numel(rows)
    rows(ii).floor = floorNo;
end
end

function rows = concatStructRows(parts)
fieldList = strings(0,1);
for pp = 1:numel(parts)
    part = parts{pp};
    if isstruct(part) && ~isempty(part)
        fieldList = unique([fieldList; string(fieldnames(part))], 'stable');
    end
end
if isempty(fieldList)
    rows = struct([]);
    return;
end
rows = repmat(blankStruct(fieldList), 0, 1);
for pp = 1:numel(parts)
    part = parts{pp};
    if ~isstruct(part) || isempty(part)
        continue;
    end
    for ii = 1:numel(part)
        item = blankStruct(fieldList);
        names = fieldnames(part(ii));
        for jj = 1:numel(names)
            item.(names{jj}) = part(ii).(names{jj});
        end
        rows(end+1,1) = item; %#ok<AGROW>
    end
end
end

function item = blankStruct(fieldList)
item = struct();
for ii = 1:numel(fieldList)
    item.(char(fieldList(ii))) = [];
end
end

function tf = statusLooksActive(payload)
tf = false;
if ~isstruct(payload)
    return;
end
tf = logicalNumberField(payload, {'hardware_moving','moving','complete'}, false) > 0;
if tf
    return;
end
mode = upper(textField(payload, 'mode', ''));
tf = mode ~= "" && mode ~= "IDLE";
end

function tf = platformParcelShouldHold(loading)
tf = false;
if ~isstruct(loading) || isempty(fieldnames(loading))
    return;
end
active = logicalNumberField(loading, 'active', false) > 0;
st = upper(textField(loading, 'state', ''));
tf = active && st == "PUSH_TO_B4_CONTACT";
end

function tf = platformParcelAcceptDetection(loading)
tf = false;
if ~isstruct(loading) || isempty(fieldnames(loading))
    return;
end
active = logicalNumberField(loading, 'active', false) > 0;
mode = upper(textField(loading, 'mode', ''));
st = upper(textField(loading, 'state', ''));
acceptStates = ["WAIT_DETECTION","ALIGN_YAW","MOVE_PLATFORM","PREPARE_B4_ENTRY_GAP","PUSH_TO_B4_CONTACT"];
tf = (active && any(st == acceptStates)) || (mode == "LOAD" && st == "LOAD_MODE_WAITING");
end

function motion = emptyMotion()
motion = struct( ...
    'active', false, ...
    'startedAt', 0.0, ...
    'belt', -1, ...
    'dir', 0, ...
    'targetMm', 0.0, ...
    'commandMm', 0.0, ...
    'rpm', 30.0, ...
    'reason', '', ...
    'handoffId', 0, ...
    'handoffReceiver', -1, ...
    'baseRows', struct([]), ...
    'lastProgress', -999.0, ...
    'key', '');
end

function ok = sameMoveCommand(cmd, event)
ok = false;
if ~isstruct(cmd) || isempty(fieldnames(cmd)) || ~isstruct(event)
    return;
end
cmdBelt = round(numberField(cmd, 'belt', -99));
evtBelt = round(numberField(event, 'belt', -98));
cmdDir = sign(numberField(cmd, 'dir', 0));
evtDir = sign(numberField(event, 'dir', 0));
cmdMm = numberField(cmd, 'mm', NaN);
evtMm = numberField(event, 'mm', NaN);
if cmdBelt ~= evtBelt || cmdDir ~= evtDir
    return;
end
if isnan(cmdMm) || isnan(evtMm)
    ok = true;
else
    ok = abs(cmdMm - evtMm) <= max(5.0, 0.10 * max(abs(cmdMm), abs(evtMm)));
end
end

function rows = rowsAfterBeltMove(rows, belt0, direction, progressMm, reason, handoffId, handoffReceiver)
if isempty(rows) || direction == 0 || progressMm <= 0
    return;
end
reasonText = string(reason);
for ii = 1:numel(rows)
    rowBelt = round(numberField(rows(ii), 'belt', -99));
    if rowBelt ~= belt0
        continue;
    end
    startPos = numberField(rows(ii), 'pos', 0);
    newPos = startPos + direction * progressMm;
    if direction > 0
        id = round(numberField(rows(ii), 'id', 0));
        axisMm = axisLengthForRow(rows(ii), belt0);
        sourceLen = beltLengthMm(belt0);
        frontAfter = startPos + axisMm / 2.0 + progressMm;
        tailAfter = startPos - axisMm / 2.0 + progressMm;
        canHandoff = handoffId > 0 && id == handoffId;
        if canHandoff && tailAfter > sourceLen
            receiver = handoffReceiver;
            if receiver < 0 || receiver > 3
                receiver = mod(belt0 + 1, 4);
            end
            excess = max(0.0, frontAfter - sourceLen);
            receiverAxis = axisLengthForRow(rows(ii), receiver);
            rows(ii).belt = receiver;
            rows(ii).pos = max(receiverAxis / 2.0, 250.0 - receiverAxis / 2.0) + excess;
        else
            rows(ii).pos = newPos;
        end
    else
        rows(ii).pos = newPos;
    end
end
if direction < 0 && contains(reasonText, "compact")
    rows = resolveReverseCompactRows(rows, belt0);
end
end

function rows = resolveReverseCompactRows(rows, belt0)
idx = [];
for ii = 1:numel(rows)
    if round(numberField(rows(ii), 'belt', -99)) == belt0
        idx(end+1) = ii; %#ok<AGROW>
    end
end
if isempty(idx)
    return;
end
centers = zeros(numel(idx), 1);
for kk = 1:numel(idx)
    centers(kk) = numberField(rows(idx(kk)), 'pos', 0);
end
[~, order] = sort(centers, 'ascend');
front = 0.0;
for oo = 1:numel(order)
    ii = idx(order(oo));
    axisMm = axisLengthForRow(rows(ii), belt0);
    center = max(numberField(rows(ii), 'pos', 0), front + axisMm / 2.0);
    rows(ii).pos = center;
    front = center + axisMm / 2.0;
end
end

function speed = motionSpeedMmSec(reason, rpm)
reasonText = string(reason);
rpm = max(1.0, double(rpm));
if contains(reasonText, "compact_reverse")
    speed = 40.0;
elseif contains(reasonText, "compact_forward")
    speed = 38.0;
elseif contains(reasonText, "tof_correction") || contains(reasonText, "gap_prepare")
    speed = 18.0;
else
    speed = max(18.0, rpm * 1.45);
end
end

function payload = decodeJsonMessage(msg)
payload = [];
try
    payload = jsondecode(char(messageText(msg)));
catch
end
end

function text = messageText(msg)
try
    text = string(msg.data);
catch
    try
        text = string(msg.Data);
    catch
        text = "";
    end
end
end

function value = numberField(s, names, defaultValue)
if nargin < 3
    defaultValue = 0;
end
value = defaultValue;
if ~isstruct(s)
    return;
end
if ischar(names) || isstring(names)
    names = cellstr(string(names));
end
for ii = 1:numel(names)
    name = char(names{ii});
    if isfield(s, name)
        raw = s.(name);
        if isempty(raw)
            continue;
        end
        try
            value = double(raw(1));
            return;
        catch
            num = str2double(string(raw(1)));
            if ~isnan(num)
                value = num;
                return;
            end
        end
    end
end
end

function value = textField(s, names, defaultValue)
if nargin < 3
    defaultValue = "";
end
value = string(defaultValue);
if ~isstruct(s)
    return;
end
if ischar(names) || isstring(names)
    names = cellstr(string(names));
end
for ii = 1:numel(names)
    name = char(names{ii});
    if isfield(s, name)
        raw = s.(name);
        if isempty(raw)
            continue;
        end
        try
            if ischar(raw)
                value = string(raw);
            elseif isstring(raw)
                value = string(raw(1));
            elseif iscell(raw)
                value = string(raw{1});
            else
                value = string(raw(1));
            end
            return;
        catch
        end
    end
end
end

function values = numericArrayField(s, name)
values = [];
if ~isstruct(s) || ~isfield(s, name)
    return;
end
raw = s.(name);
try
    values = double(raw(:)');
catch
end
end

function s = structField(parent, name)
s = struct();
if isstruct(parent) && isfield(parent, name) && isstruct(parent.(name))
    s = parent.(name);
end
end

function value = logicalNumberField(s, names, defaultValue)
value = double(logical(defaultValue));
if ~isstruct(s)
    return;
end
if ischar(names) || isstring(names)
    names = cellstr(string(names));
end
for ii = 1:numel(names)
    name = char(names{ii});
    if ~isfield(s, name)
        continue;
    end
    raw = s.(name);
    if isempty(raw)
        continue;
    end
    try
        if ischar(raw) || isstring(raw)
            text = lower(string(raw(1)));
            value = double(text == "true" || text == "1" || text == "yes" || text == "on");
        else
            value = double(logical(raw(1)));
        end
        return;
    catch
    end
end
end

function val = safeIndex(values, idx)
if numel(values) >= idx
    val = values(idx);
else
    val = NaN;
end
end

function rows = sortDbRows(rows)
if isempty(rows)
    return;
end
seq = zeros(numel(rows),1);
ids = zeros(numel(rows),1);
for ii = 1:numel(rows)
    seq(ii) = numberField(rows(ii), 'seq', 9999);
    ids(ii) = numberField(rows(ii), 'id', 0);
end
[~, order] = sortrows([seq ids]);
rows = rows(order);
end

function sig = dbSignature(rows)
if isempty(rows)
    sig = "";
    return;
end
rows = sortDbRows(rows);
parts = strings(numel(rows),1);
for ii = 1:numel(rows)
    parts(ii) = sprintf('%d:%d:%d:%.1f:%.1f:%.1f', ...
        round(numberField(rows(ii), 'floor', 1)), ...
        round(numberField(rows(ii), 'id', 0)), ...
        round(numberField(rows(ii), 'belt', -1)), ...
        numberField(rows(ii), 'pos', 0), ...
        numberField(rows(ii), {'long_side','longSide','long'}, 0), ...
        numberField(rows(ii), {'short_side','shortSide','short'}, 0));
end
sig = strjoin(parts, '|');
end

function sig = statusRenderSignature(status)
sig = "";
if ~isstruct(status) || isempty(fieldnames(status))
    return;
end
platform = structField(status, 'platform');
pusher = structField(status, 'pusher');
platformParcel = structField(status, 'platform_parcel');
unload = structField(status, 'unload');
digitalTwin = structField(status, 'digital_twin');
parts = strings(0,1);
if isstruct(platform)
    parts(end+1,1) = sprintf('pf:%d:%.1f:%.1f:%.1f:%.1f:%d:%s', ...
        round(numberField(platform, 'floor', 1)), ...
        numberField(platform, 'z_mm', 0), ...
        numberField(platform, {'tilt_deg','target_tilt_deg','angle_deg'}, 0), ...
        numberField(platform, 'z_uncertainty_mm', 0), ...
        numberField(platform, 'tilt_uncertainty_deg', 0), ...
        round(numberField(platform, 'box_id', 0)), ...
        char(textField(platform, 'confidence', '')));
end
if isstruct(pusher)
    parts(end+1,1) = sprintf('ps:%.1f:%.1f:%.1f:%.1f:%d:%d:%s', ...
        numberField(pusher, {'main_mm','pusher_mm','main'}, 0), ...
        numberField(pusher, {'side_mm','side'}, 0), ...
        numberField(pusher, 'main_uncertainty_mm', 0), ...
        numberField(pusher, 'side_uncertainty_mm', 0), ...
        logicalNumberField(pusher, {'main_active','pusher_active'}, false), ...
        logicalNumberField(pusher, {'side_active'}, false), ...
        char(textField(pusher, 'confidence', '')));
end
if isstruct(platformParcel) && ~isempty(fieldnames(platformParcel))
    parts(end+1,1) = sprintf('pp:%d:%d:%.1f:%.1f:%.1f:%.1f:%d', ...
        logicalNumberField(platformParcel, 'visible', false), ...
        round(numberField(platformParcel, {'type','box_type'}, 0)), ...
        numberField(platformParcel, {'long_side','long_mm','long'}, 0), ...
        numberField(platformParcel, {'short_side','short_mm','short'}, 0), ...
        numberField(platformParcel, 'pusher_mm', 0), ...
        numberField(platformParcel, 'yaw_deg', 0), ...
        round(numberField(platformParcel, 'floor', 1)));
end
if isstruct(unload)
    count = 0;
    if isfield(unload, 'packages') && isstruct(unload.packages)
        count = numel(unload.packages);
        packageParts = strings(count,1);
        for ii = 1:count
            pkg = unload.packages(ii);
            packageParts(ii) = sprintf('u:%d:%d:%.1f:%.1f:%s', ...
                round(numberField(pkg, 'id', 0)), ...
                round(numberField(pkg, 'floor', 0)), ...
                numberField(pkg, 'slot_mm', 0), ...
                numberField(pkg, 'uncertainty_mm', 0), ...
                char(textField(pkg, 'confidence', '')));
        end
        parts = [parts; packageParts];
    end
    parts(end+1,1) = sprintf('uc:%d:%.1f:%s', count, ...
        numberField(unload, 'layout_uncertainty_mm', 0), ...
        char(textField(unload, 'confidence', '')));
end
if isstruct(digitalTwin) && isfield(digitalTwin, 'warnings')
    try
        warnings = string(digitalTwin.warnings);
        parts(end+1,1) = "dtw:" + strjoin(warnings(:)', ',');
    catch
    end
end
sig = strjoin(parts, '|');
end

function sig = loadingStateSignature(payload)
sig = "";
if ~isstruct(payload) || isempty(fieldnames(payload))
    return;
end
det = structField(payload, 'latest_detection');
tuning = structField(payload, 'pusher_tuning');
barrier = structField(payload, 'barrier_tuning');
barrierStates = structField(payload, 'barrier_states');
sig = sprintf('%s:%s:%d:%s:%d:%d:%d:%.1f:%.1f:%.1f:%.1f:%.1f:%s:%s:%s:%.1f:%.1f', ...
    char(textField(payload, 'mode', '-')), ...
    char(textField(payload, 'state', '-')), ...
    logicalNumberField(payload, 'active', false), ...
    char(textField(payload, 'last_error', '')), ...
    logicalNumberField(det, 'present', false), ...
    round(numberField(det, 'parcel_type', 0)), ...
    round(numberField(payload, 'target_floor', 0)), ...
    numberField(tuning, 'contact_to_b4_mm', NaN), ...
    numberField(tuning, 'extra_mm', NaN), ...
    numberField(tuning, 'b4_assist_mm', NaN), ...
    numberField(payload, 'current_floor_z_mm', NaN), ...
    numberField(payload, 'current_floor_offset_mm', NaN), ...
    char(textField(payload, 'barrier_state', '-')), ...
    char(textField(barrierStates, {'x1','1'}, '-')), ...
    char(textField(barrierStates, {'x2','2'}, '-')), ...
    numberField(barrier, 'up_angle_deg', NaN), ...
    numberField(barrier, 'down_angle_deg', NaN));
end

function text = loadingStateText(payload)
if ~isstruct(payload) || isempty(fieldnames(payload))
    text = "not received";
    return;
end
det = structField(payload, 'latest_detection');
tuning = structField(payload, 'pusher_tuning');
barrier = structField(payload, 'barrier_tuning');
barrierStates = structField(payload, 'barrier_states');
present = logicalNumberField(det, 'present', false);
parcelType = round(numberField(det, 'parcel_type', 0));
longMm = numberField(det, 'long_mm', NaN);
shortMm = numberField(det, 'short_mm', NaN);
errorText = textField(payload, 'last_error', '');
if errorText ~= ""
    errorText = " err=" + errorText;
end
detectText = sprintf('present=%d type=%d', present, parcelType);
if ~isnan(longMm) && ~isnan(shortMm)
    detectText = sprintf('%s %.0fx%.0f', detectText, longMm, shortMm);
end
tuneText = "";
if isstruct(tuning) && ~isempty(fieldnames(tuning))
    tuneText = sprintf(' push[base %.1f extra %.1f assist %.1f]', ...
        numberField(tuning, 'contact_to_b4_mm', NaN), ...
        numberField(tuning, 'extra_mm', NaN), ...
        numberField(tuning, 'b4_assist_mm', NaN));
end
liftText = "";
zMm = numberField(payload, 'current_floor_z_mm', NaN);
offsetMm = numberField(payload, 'current_floor_offset_mm', NaN);
baseMm = numberField(payload, 'current_floor_base_z_mm', NaN);
if ~isnan(zMm)
    liftText = sprintf(' lift[z %.1f offset %+0.1f base %.1f]', zMm, offsetMm, baseMm);
end
barrierText = "";
if isstruct(barrier) && ~isempty(fieldnames(barrier))
    barrierText = sprintf(' barrier[F1 %s F2 %s up %.0f down %.0f]', ...
        char(textField(barrierStates, {'x1','1'}, textField(payload, 'barrier_state', '-'))), ...
        char(textField(barrierStates, {'x2','2'}, '-')), ...
        numberField(barrier, 'up_angle_deg', NaN), ...
        numberField(barrier, 'down_angle_deg', NaN));
end
text = sprintf('mode=%s state=%s active=%d floor=%d %s%s%s%s%s', ...
    char(textField(payload, 'mode', '-')), ...
    char(textField(payload, 'state', '-')), ...
    logicalNumberField(payload, 'active', false), ...
    round(numberField(payload, 'target_floor', 0)), ...
    detectText, char(liftText), char(tuneText), char(barrierText), char(errorText));
end

function item = firstStruct(value)
item = struct();
if isstruct(value) && ~isempty(value)
    item = value(1);
end
end

function text = compactStructText(s)
try
    names = fieldnames(s);
    parts = strings(0,1);
    for ii = 1:min(numel(names), 6)
        name = names{ii};
        if strcmp(name, 'level')
            continue;
        end
        raw = s.(name);
        if isnumeric(raw) || islogical(raw)
            val = sprintf('%g', double(raw(1)));
        elseif ischar(raw)
            val = char(string(raw));
        elseif isstring(raw)
            val = char(string(raw(1)));
        elseif iscell(raw) && ~isempty(raw)
            val = char(string(raw{1}));
        else
            continue;
        end
        parts(end+1,1) = string(name) + "=" + string(val); %#ok<AGROW>
    end
    text = strjoin(parts, ' ');
catch
    text = "";
end
end

function text = dirText(direction)
if direction > 0
    text = '+ forward';
elseif direction < 0
    text = '- reverse';
else
    text = 'dir 0';
end
end

function len = beltLengthMm(belt0)
lengths = [498.0 1080.0 498.0 1080.0];
idx = round(belt0) + 1;
if idx < 1 || idx > numel(lengths)
    len = 1000.0;
else
    len = lengths(idx);
end
end

function axisMm = axisLengthForRow(row, belt0)
longSide = numberField(row, {'long_side','longSide','long'}, 0);
shortSide = numberField(row, {'short_side','shortSide','short'}, 0);
if belt0 == 0 || belt0 == 2
    axisMm = longSide;
else
    axisMm = shortSide;
end
axisMm = max(axisMm, 1.0);
end

function t = posixNow()
t = posixtime(datetime('now', 'TimeZone', 'local'));
end

function tf = sameText(a, b)
try
    aa = string(a);
    bb = string(b);
    if isempty(aa)
        aa = "";
    end
    if isempty(bb)
        bb = "";
    end
    tf = strcmp(char(aa(1)), char(bb(1)));
catch
    tf = strcmp(char(string(a)), char(string(b)));
end
end

function n = speedSteps()
vals = [1 4 8 16 32];
figs = findall(0, 'Type', 'figure', 'Name', 'Refuge MATLAB Digital Twin Panel');
if isempty(figs)
    n = 4;
    return;
end
popup = findobj(figs(1), 'Style', 'popupmenu');
if isempty(popup)
    n = 4;
    return;
end
idx = popup(1).Value;
n = max(1, round(vals(idx) * 0.05 / 0.01));
end

function line = statusLine(S)
if isfield(S, 'circCompleteTargetId') && S.circCompleteTargetId > 0
    line = sprintf('Local DONE P%d | Loaded %d', S.circCompleteTargetId, S.loadedCount);
elseif isfield(S, 'pendingUnloadId') && S.pendingUnloadId > 0
    line = sprintf('Local queued P%d | Loaded %d', S.pendingUnloadId, S.loadedCount);
else
    line = sprintf('Local %s | Loaded %d', statusTextLocal(S.statusCode), S.loadedCount);
end
end

function s = statusTextLocal(code)
if code == 0
    s = 'IDLE';
elseif code == 1
    s = 'LOADING';
elseif code == 2
    s = 'CIRCULATION';
elseif code == 3
    s = 'UNLOADING';
else
    s = 'COMPLETE';
end
end

function pos = visibleControlPosition(w, h)
screen = get(0, 'ScreenSize');
margin = 30;
left = screen(1) + max(margin, screen(3) - w - margin);
bottom = screen(2) + max(margin, screen(4) - h - 90);
left = min(left, screen(1) + screen(3) - w - margin);
bottom = min(bottom, screen(2) + screen(4) - h - margin);
left = max(left, screen(1) + margin);
bottom = max(bottom, screen(2) + margin);
pos = [left bottom w h];
end
