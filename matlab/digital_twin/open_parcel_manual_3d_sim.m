function open_parcel_manual_3d_sim()
%OPEN_PARCEL_MANUAL_3D_SIM Launch manual load/unload 3D simulation controls.
%
% Buttons:
%   Load Package       Generate one random Korea Post parcel and load it.
%   Unload Target      Circulate the requested package ID to Belt4 and unload it.
%   Reset              Clear simulation state.

rootDir = fileparts(mfilename('fullpath'));
addpath(rootDir);

parcel_manual_animation_update([]);
S = parcel_manual_core_step("reset", 0);
parcel_manual_animation_update(S);

ctrl = figure('Name', 'Parcel Manual Controls', ...
    'Color', [0.96 0.97 0.98], ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'NumberTitle', 'off', ...
        'Position', visibleControlPosition(360, 455), ...
    'CloseRequestFcn', @onClose);

uicontrol(ctrl, 'Style', 'text', ...
    'String', 'Manual Simulation Control', ...
    'FontWeight', 'bold', 'FontSize', 12, ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', ...
    'Position', [18 415 300 24]);

statusLabel = uicontrol(ctrl, 'Style', 'text', ...
    'String', 'IDLE', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', ...
    'Position', [18 390 320 22]);

uicontrol(ctrl, 'Style', 'text', ...
    'String', 'Seq DB Input', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', ...
    'Position', [18 355 100 20]);

floorPopup = uicontrol(ctrl, 'Style', 'popupmenu', ...
    'String', {'F1','F2','F3'}, ...
    'Value', 1, ...
    'Position', [18 325 58 27]);

longEdit = uicontrol(ctrl, 'Style', 'edit', ...
    'String', '220', ...
    'Position', [84 325 55 27]);

shortEdit = uicontrol(ctrl, 'Style', 'edit', ...
    'String', '180', ...
    'Position', [146 325 55 27]);

heightEdit = uicontrol(ctrl, 'Style', 'edit', ...
    'String', '90', ...
    'Position', [208 325 45 27]);

uicontrol(ctrl, 'Style', 'pushbutton', ...
    'String', 'Seq Add', ...
    'FontWeight', 'bold', ...
    'Position', [263 325 78 29], ...
    'Callback', @onSeqAdd);

uicontrol(ctrl, 'Style', 'pushbutton', ...
    'String', 'Load Package', ...
    'FontWeight', 'bold', ...
    'Position', [18 275 135 34], ...
    'Callback', @onLoad);

uicontrol(ctrl, 'Style', 'pushbutton', ...
    'String', 'Load To Wait Area', ...
    'FontWeight', 'bold', ...
    'Position', [165 275 176 34], ...
    'Callback', @onWaitLoad);

uicontrol(ctrl, 'Style', 'pushbutton', ...
    'String', 'Demo Layout', ...
    'Position', [18 235 135 30], ...
    'Callback', @onDemoLayout);

uicontrol(ctrl, 'Style', 'text', ...
    'String', 'Target ID', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', ...
    'Position', [175 240 60 20]);

targetEdit = uicontrol(ctrl, 'Style', 'edit', ...
    'String', '1', ...
    'Position', [238 237 62 27]);

uicontrol(ctrl, 'Style', 'pushbutton', ...
    'String', 'Start Circulation', ...
    'FontWeight', 'bold', ...
    'Position', [18 185 150 34], ...
    'Callback', @onUnload);

uicontrol(ctrl, 'Style', 'pushbutton', ...
    'String', 'Reset', ...
    'Position', [188 185 125 34], ...
    'Callback', @onReset);

uicontrol(ctrl, 'Style', 'pushbutton', ...
    'String', 'Release Wait Box', ...
    'Position', [18 145 135 30], ...
    'Callback', @onClearWait);

uicontrol(ctrl, 'Style', 'text', ...
    'String', 'Speed', ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', ...
    'Position', [175 150 45 20]);

speedPopup = uicontrol(ctrl, 'Style', 'popupmenu', ...
    'String', {'1x','4x','8x','16x','32x'}, ...
    'Value', 2, ...
    'Position', [220 146 80 27]);

helpText = sprintf(['SEQ: floor, long, short, height [mm]\n' ...
    'Start: target reaches B3->B4 unload zone\n' ...
    'Refuge: B4 reverse drop + forward restore\n' ...
    'Speed controls playback only']);
uicontrol(ctrl, 'Style', 'text', ...
    'String', helpText, ...
    'BackgroundColor', [0.96 0.97 0.98], ...
    'HorizontalAlignment', 'left', ...
    'Position', [18 20 320 92]);

timerPeriod = 0.04;
coreTs = 0.01;
tmr = timer('ExecutionMode', 'fixedSpacing', ...
    'BusyMode', 'drop', ...
    'Period', timerPeriod, ...
    'TimerFcn', @onTick);
start(tmr);

    function onTick(~, ~)
        if ~isvalid(ctrl)
            return;
        end
        steps = speedSteps();
        for k = 1:steps
            S = parcel_manual_core_step("step", 0);
        end
        parcel_manual_animation_update(S);
        statusLabel.String = statusLine(S);
    end

    function onSeqAdd(~, ~)
        S = parcel_manual_core_step("snapshot", 0);
        if ~S.isIdle
            statusLabel.String = 'Busy: wait until current motion finishes';
            return;
        end
        floor = floorPopup.Value;
        longSide = str2double(longEdit.String);
        shortSide = str2double(shortEdit.String);
        height = str2double(heightEdit.String);
        if any(isnan([longSide shortSide height])) || any([longSide shortSide height] <= 0)
            statusLabel.String = 'Enter long / short / height in mm';
            return;
        end
        S = parcel_manual_core_step("seq_package", [floor longSide shortSide height]);
        parcel_manual_animation_update(S);
        statusLabel.String = char(S.message);
        if S.loadedCount > 0
            targetEdit.String = sprintf('%d', S.loadedCount);
        end
    end

    function onDemoLayout(~, ~)
        S = parcel_manual_core_step("snapshot", 0);
        if ~S.isIdle
            statusLabel.String = 'Busy: wait until current motion finishes';
            return;
        end
        S = parcel_manual_core_step("reset", 0);
        nextTarget = 2;
        for floor = 1:3
            for n = 1:15
                idHint = (floor - 1) * 15 + n;
                dims = demoDims(idHint);
                S = parcel_manual_core_step("seq_package", [floor dims]);
                if startsWith(string(S.message), "SEQ BLOCKED")
                    break;
                end
            end
        end
        targetEdit.String = sprintf('%d', nextTarget);
        parcel_manual_animation_update(S);
        statusLabel.String = sprintf('Demo layout ready | target P%d', nextTarget);
    end

    function onLoad(~, ~)
        S = parcel_manual_core_step("snapshot", 0);
        if isfield(S, 'platformReadyForLoad')
            platformReady = S.platformReadyForLoad;
        else
            platformReady = S.isIdle;
        end
        if ~platformReady
            statusLabel.String = 'Busy: wait until current motion finishes';
            return;
        end
        S = parcel_manual_core_step("load", 0);
        if S.phase == 0 && startsWith(string(S.message), "LOAD BLOCKED")
            statusLabel.String = char(S.message);
        end
        parcel_manual_animation_update(S);
    end

    function onWaitLoad(~, ~)
        S = parcel_manual_core_step("snapshot", 0);
        if isfield(S, 'platformReadyForLoad')
            platformReady = S.platformReadyForLoad;
        else
            platformReady = S.isIdle;
        end
        if ~platformReady
            statusLabel.String = 'Busy: wait until current motion finishes';
            return;
        end
        S = parcel_manual_core_step("wait_load", 0);
        parcel_manual_animation_update(S);
        statusLabel.String = char(S.message);
    end

    function onUnload(~, ~)
        S = parcel_manual_core_step("snapshot", 0);
        if S.phase ~= 0
            statusLabel.String = 'Busy: wait until current motion finishes';
            return;
        end
        targetId = str2double(targetEdit.String);
        if isnan(targetId) || targetId < 1
            statusLabel.String = 'Enter a valid package ID';
            return;
        end
        S = parcel_manual_core_step("unload", round(targetId));
        if S.targetId == 0
            statusLabel.String = sprintf('Package %d is not loaded', round(targetId));
        end
        parcel_manual_animation_update(S);
    end

    function onReset(~, ~)
        S = parcel_manual_core_step("reset", 0);
        parcel_manual_animation_update(S);
        statusLabel.String = 'IDLE';
    end

    function onClearWait(~, ~)
        S = parcel_manual_core_step("clear_wait", 0);
        parcel_manual_animation_update(S);
        statusLabel.String = char(S.message);
    end

    function line = statusLine(S)
        refuge = 0;
        if isfield(S, 'tempUnloadCount')
            refuge = S.tempUnloadCount;
        end
        done = 0;
        if isfield(S, 'circCompleteTargetId')
            done = S.circCompleteTargetId;
        end
        if done > 0
            line = sprintf('DONE P%d | Loaded %d | Refuge %d', done, S.loadedCount, refuge);
        elseif isfield(S, 'pendingUnloadId') && S.pendingUnloadId > 0
            line = sprintf('QUEUED P%d | finishing load route | Loaded %d', ...
                S.pendingUnloadId, S.loadedCount);
        else
            line = sprintf('%s | Loaded %d | Refuge %d', ...
                statusTextLocal(S.statusCode), S.loadedCount, refuge);
        end
    end

    function dims = demoDims(id)
        longSet = [115 135 155 175 205 230 245];
        shortSet = [85 95 110 125 145 165 185];
        k = 1 + mod(id * 3 + 5, numel(longSet));
        m = 1 + mod(id * 5 + 2, numel(shortSet));
        longSide = longSet(k);
        shortSide = min(shortSet(m), longSide - 10);
        height = 80 + 10 * mod(id, 5);
        dims = [longSide shortSide height];
    end

    function n = speedSteps()
        vals = [1 4 8 16 32];
        idx = speedPopup.Value;
        n = max(1, round(vals(idx) * timerPeriod / coreTs));
    end

    function onClose(~, ~)
        if exist('tmr', 'var') && isa(tmr, 'timer') && isvalid(tmr)
            stop(tmr);
            delete(tmr);
        end
        if isvalid(ctrl)
            delete(ctrl);
        end
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
left = screen(1) + max(margin, (screen(3) - w) * 0.5);
bottom = screen(2) + max(margin, screen(4) - h - 90);
left = min(left, screen(1) + screen(3) - w - margin);
bottom = min(bottom, screen(2) + screen(4) - h - margin);
left = max(left, screen(1) + margin);
bottom = max(bottom, screen(2) + margin);
pos = [left bottom w h];
end
