import { ArmedLateralMode, ArmedVerticalMode, isArmed, LateralMode, VerticalMode } from '@shared/autopilot.js';
import React, { Component } from 'react';
import { createDeltaTimeCalculator, getSimVar, renderTarget } from '../util.js';

export const FMA = ({ isAttExcessive }) => {
    const activeLateralMode = getSimVar('L:A32NX_FMA_LATERAL_MODE', 'number');
    const activeVerticalMode = getSimVar('L:A32NX_FMA_VERTICAL_MODE', 'enum');
    const sharedModeActive = activeLateralMode === LateralMode.LAND || activeLateralMode === LateralMode.FLARE || activeLateralMode === LateralMode.ROLL_OUT
        || (activeLateralMode === LateralMode.NAV && activeVerticalMode === VerticalMode.FINAL);
    const engineMessage = getSimVar('L:A32NX_AUTOTHRUST_MODE_MESSAGE', 'enum');
    const BC3Message = getBC3Message(isAttExcessive)[0] !== null;
    const AB3Message = (getSimVar('L:A32NX_MachPreselVal', 'mach') !== -1
        || getSimVar('L:A32NX_SpeedPreselVal', 'knots') !== -1) && !BC3Message && engineMessage === 0;

    let secondBorder: string;
    if (sharedModeActive && !isAttExcessive) {
        secondBorder = '';
    } else if (BC3Message) {
        secondBorder = 'm66.241 0.33732v15.766';
    } else {
        secondBorder = 'm66.241 0.33732v20.864';
    }

    let firstBorder: string;
    if (AB3Message && !isAttExcessive) {
        firstBorder = 'm33.117 0.33732v15.766';
    } else {
        firstBorder = 'm33.117 0.33732v20.864';
    }

    return (
        <g id="FMA">
            <g className="NormalStroke Grey">
                <path d={firstBorder} />
                <path d={secondBorder} />
                <path d="m102.52 0.33732v20.864" />
                <path d="m133.72 0.33732v20.864" />
            </g>

            <Row1 isAttExcessive={isAttExcessive} />
            <Row2 isAttExcessive={isAttExcessive} />
            <Row3 isAttExcessive={isAttExcessive} />
        </g>
    );
};

const Row1 = ({ isAttExcessive }) => (
    <g>
        <A1A2Cell />
        {!isAttExcessive && (
            <>
                <B1Cell />
                <C1Cell />
                <D1D2Cell />
                <BC1Cell />
            </>
        )}
        <E1Cell />
    </g>
);

const Row2 = ({ isAttExcessive }) => (
    <g>
        {!isAttExcessive && (
            <>
                <B2Cell />
                <C2Cell />
            </>
        )}
        <E2Cell />
    </g>
);

const Row3 = ({ isAttExcessive }) => (
    <g>
        <A3Cell />
        {!isAttExcessive && (
            <>
                <AB3Cell />
                <D3Cell />
            </>
        )}
        <BC3Cell isAttExcessive={isAttExcessive} />
        <E3Cell />
    </g>
);

const A1A2Cell = () => {
    const AThrMode = getSimVar('L:A32NX_AUTOTHRUST_MODE', 'enum');

    let text: string | undefined;

    switch (AThrMode) {
    case 1:
        return (
            <g>
                <path className="NormalStroke White" d="m25.114 1.8143v13.506h-16.952v-13.506z" />
                <text className="FontMedium MiddleAlign White" x="16.782249" y="7.1280665">MAN</text>
                <text className="FontMedium MiddleAlign White" x="16.869141" y="14.351689">TOGA</text>
            </g>
        );
    case 2:
        return (
            <g>
                <path className="NormalStroke White" d="m31.521 1.8143v13.506h-30.217v-13.506z" />
                <text className="FontMedium MiddleAlign White" x="16.782249" y="7.1280665">MAN</text>
                <text className="FontMedium MiddleAlign White" x="16.869141" y="14.351689">GA SOFT</text>
            </g>
        );
    case 3: {
        const FlexTemp = Math.round(getSimVar('L:AIRLINER_TO_FLEX_TEMP', 'number'));
        const FlexText = FlexTemp >= 0 ? (`+${FlexTemp}`) : FlexTemp.toString();
        return (
            <g>
                <path className="NormalStroke White" d="m31.521 1.8143v13.506h-30.217v-13.506z" />
                <text className="FontMedium MiddleAlign White" x="16.782249" y="7.1280665">MAN</text>
                <text className="FontMedium MiddleAlign White" x="16.869141" y="14.351689">
                    <tspan xmlSpace="preserve">FLX  </tspan>
                    <tspan className="Cyan">{FlexText}</tspan>
                </text>
            </g>
        );
    }
    case 4:
        return (
            <g>
                <path className="NormalStroke White" d="m25.114 1.8143v13.506h-16.952v-13.506z" />
                <text className="FontMedium MiddleAlign White" x="16.782249" y="7.1280665">MAN</text>
                <text className="FontMedium MiddleAlign White" x="16.869141" y="14.351689">DTO</text>
            </g>
        );
    case 5:
        return (
            <g>
                <path className="NormalStroke White" d="m25.114 1.8143v13.506h-16.952v-13.506z" />
                <text className="FontMedium MiddleAlign White" x="16.782249" y="7.1280665">MAN</text>
                <text className="FontMedium MiddleAlign White" x="16.869141" y="14.351689">MCT</text>
            </g>
        );
    case 6:
        return (
            <g>
                <path className="NormalStroke Amber" d="m25.114 1.8143v13.506h-16.952v-13.506z" />
                <text className="FontMedium MiddleAlign White" x="16.782249" y="7.1280665">MAN</text>
                <text className="FontMedium MiddleAlign White" x="16.869141" y="14.351689">THR</text>
            </g>
        );
    case 7:
        text = 'SPEED';
        break;
    case 8:
        text = 'MACH';
        break;
    case 9:
        text = 'THR MCT';
        break;
    case 10:
        text = 'THR CLB';
        break;
    case 11:
        text = 'THR LVR';
        break;
    case 12:
        text = 'THR IDLE';
        break;
    case 13:
        return (
            <g>
                <path className="NormalStroke Amber BlinkInfinite" d="m0.70556 1.8143h30.927v6.0476h-30.927z" />
                <text className="FontMedium MiddleAlign Green" x="16.782249" y="7.1280665">A.FLOOR</text>
            </g>
        );
    case 14:
        return (
            <g>
                <path className="NormalStroke Amber BlinkInfinite" d="m0.70556 1.8143h30.927v6.0476h-30.927z" />
                <text className="FontMedium MiddleAlign Green" x="16.782249" y="7.1280665">TOGA LK</text>
            </g>
        );
    default:
        return null;
    }

    return (
        <g>
            <ShowForSeconds timer={9} id={AThrMode}>
                <path className="NormalStroke White" d="m0.70556 1.8143h30.927v6.0476h-30.927z" />
            </ShowForSeconds>
            <text className="FontMedium MiddleAlign Green" x="16.782249" y="7.1280665">{text}</text>
        </g>
    );
};

const A3Cell = () => {
    const engineMessage = getSimVar('L:A32NX_AUTOTHRUST_MODE_MESSAGE', 'enum');

    let text: string;
    let className: string;
    switch (engineMessage) {
    case 1:
        text = 'THR LK';
        className = 'Amber BlinkInfinite';
        break;
    case 2:
        text = 'LVR TOGA';
        className = 'White BlinkInfinite';
        break;
    case 3:
        text = 'LVR CLB';
        className = 'White BlinkInfinite';
        break;
    case 4:
        text = 'LVR MCT';
        className = 'White BlinkInfinite';
        break;
    case 5:
        text = 'LVR ASYM';
        className = 'Amber';
        break;
    default:
        return null;
    }

    return (
        <text className={`FontMedium MiddleAlign ${className}`} x="16.989958" y="21.641243">{text}</text>
    );
};

const AB3Cell = () => {
    if (getSimVar('L:A32NX_AUTOTHRUST_MODE_MESSAGE', 'enum') !== 0) {
        return null;
    }
    const machPresel = getSimVar('L:A32NX_MachPreselVal', 'mach');
    if (machPresel !== -1) {
        const text = machPresel.toFixed(2);
        return (
            <text className="FontMedium MiddleAlign Cyan" x="35.434673" y="21.656223">{`MACH SEL ${text}`}</text>
        );
    }
    const spdPresel = getSimVar('L:A32NX_SpeedPreselVal', 'knots');
    if (spdPresel !== -1) {
        const text = Math.round(spdPresel);
        return (
            <text className="FontMedium MiddleAlign Cyan" x="35.434673" y="21.656223">{`SPEED SEL ${text}`}</text>
        );
    }
    return null;
};

const B1Cell = () => {
    const activeVerticalMode = getSimVar('L:A32NX_FMA_VERTICAL_MODE', 'enum');

    let text: string | JSX.Element;
    let inProtection = false;

    switch (activeVerticalMode) {
    case VerticalMode.GS_TRACK:
        text = 'G/S';
        break;
    // case 2:
    //     text = 'F-G/S';
    //     break;
    case VerticalMode.GS_CPT:
        text = 'G/S*';
        break;
    // case 4:
    //     text = 'F-G/S*';
    //     break;
    case VerticalMode.SRS:
    case VerticalMode.SRS_GA:
        text = 'SRS';
        break;
    case VerticalMode.TCAS:
        text = 'TCAS';
        break;
    // case 9:
    //     text = 'FINAL';
    //     break;
    case VerticalMode.DES:
        text = 'DES';
        break;
    case VerticalMode.OP_DES:
        if (getSimVar('L:A32NX_FMA_EXPEDITE_MODE', 'bool')) {
            text = 'EXP DES';
        } else {
            text = 'OP DES';
        }
        break;
    case VerticalMode.CLB:
        text = 'CLB';
        break;
    case VerticalMode.OP_CLB:
        if (getSimVar('L:A32NX_FMA_EXPEDITE_MODE', 'bool')) {
            text = 'EXP CLB';
        } else {
            text = 'OP CLB';
        }
        break;
    case VerticalMode.ALT:
        if (getSimVar('L:A32NX_FMA_CRUISE_ALT_MODE', 'Bool')) {
            text = 'ALT CRZ';
        } else {
            text = 'ALT';
        }
        break;
    case VerticalMode.ALT_CPT:
        text = 'ALT*';
        break;
    case VerticalMode.ALT_CST:
        text = 'ALT CST';
        break;
    case VerticalMode.ALT_CST_CPT:
        text = 'ALT CST*';
        break;
    case VerticalMode.FPA: {
        const FPA = getSimVar('L:A32NX_AUTOPILOT_FPA_SELECTED', 'Degree');
        inProtection = getSimVar('L:A32NX_FMA_SPEED_PROTECTION_MODE', 'bool');
        const FPAText = `${(FPA >= 0 ? '+' : '')}${(Math.round(FPA * 10) / 10).toFixed(1)}°`;

        text = (
            <>
                <tspan>FPA</tspan>
                <tspan className={`${inProtection ? 'PulseCyanFill' : 'Cyan'}`} xmlSpace="preserve">{FPAText}</tspan>
            </>
        );
        break;
    }
    case VerticalMode.VS: {
        const VS = getSimVar('L:A32NX_AUTOPILOT_VS_SELECTED', 'feet per minute');
        inProtection = getSimVar('L:A32NX_FMA_SPEED_PROTECTION_MODE', 'bool');
        const VSText = `${(VS >= 0 ? '+' : '')}${Math.round(VS).toString()}`.padStart(5, ' ');

        text = (
            <>
                <tspan>V/S</tspan>
                <tspan className={`${inProtection ? 'PulseCyanFill' : 'Cyan'}`} xmlSpace="preserve">{VSText}</tspan>
            </>
        );
        break;
    }
    default:
        return null;
    }

    const inSpeedProtection = inProtection && (activeVerticalMode === VerticalMode.VS || activeVerticalMode === VerticalMode.FPA);
    const inModeReversion = getSimVar('L:A32NX_FMA_MODE_REVERSION', 'bool');

    const tcasModeDisarmedMessage = getSimVar('L:A32NX_AUTOPILOT_TCAS_MESSAGE_DISARM', 'bool');

    const boxClassName = (inSpeedProtection || inModeReversion) ? 'NormalStroke None' : 'NormalStroke White';
    const boxPathString = activeVerticalMode === VerticalMode.TCAS && tcasModeDisarmedMessage ? 'm34.656 1.8143h29.918v13.506h-29.918z' : 'm34.656 1.8143h29.918v6.0476h-29.918z';

    return (
        <g>
            <ShowForSeconds timer={10} id={activeVerticalMode}>
                <path className={boxClassName} d={boxPathString} />
            </ShowForSeconds>
            {inSpeedProtection && <path className="NormalStroke Amber BlinkInfinite" d="m34.656 1.8143h29.918v6.0476h-29.918z" />}
            {inModeReversion && <path className="NormalStroke White BlinkInfinite" d="m34.656 1.8143h29.918v6.0476h-29.918z" />}
            <text className="FontMedium MiddleAlign Green" x="49.921795" y="7.1040988" xmlSpace="preserve">{text}</text>
        </g>
    );
};

const B2Cell = () => {
    const armedVerticalBitmask = getSimVar('L:A32NX_FMA_VERTICAL_ARMED', 'number');

    const altArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.ALT);
    const altCstArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.ALT_CST);
    const clbArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.CLB);
    const desArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.DES);
    const gsArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.GS);
    const finalArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.FINAL);

    let text1: string | null;
    let color1 = 'Cyan';
    if (clbArmed) {
        text1 = 'CLB';
    } else if (desArmed) {
        text1 = 'DES';
    } else if (altCstArmed) {
        text1 = 'ALT';
        color1 = 'Magenta';
    } else if (altArmed) {
        text1 = 'ALT';
    } else {
        text1 = null;
    }

    let text2;
    // case 1:
    //     text2 = 'F-G/S';
    //     break;
    if (gsArmed) {
        text2 = 'G/S';
    } else if (finalArmed) {
        text2 = 'FINAL';
    } else {
        text2 = null;
    }

    return (
        <g>
            {text1
                && <text className={`FontMedium MiddleAlign ${color1}`} x="41.477474" y="14.329653">{text1}</text>}
            {text2
                && <text className="FontMedium MiddleAlign Cyan" x="54.59803" y="14.382949">{text2}</text>}
        </g>
    );
};

const C1Cell = () => {
    const activeLateralMode = getSimVar('L:A32NX_FMA_LATERAL_MODE', 'number');

    const armedVerticalBitmask = getSimVar('L:A32NX_FMA_VERTICAL_ARMED', 'number');
    const finalArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.FINAL);

    const activeVerticalMode = getSimVar('L:A32NX_FMA_VERTICAL_MODE', 'enum');

    let text: string;
    let id = 0;
    if (activeLateralMode === LateralMode.GA_TRACK) {
        text = 'GA TRK';
        id = 1;
    } else if (activeLateralMode === LateralMode.LOC_CPT) {
        text = 'LOC *';
        id = 3;
    } else if (activeLateralMode === LateralMode.HDG) {
        text = 'HDG';
        id = 5;
    } else if (activeLateralMode === LateralMode.RWY) {
        text = 'RWY';
        id = 6;
    } else if (activeLateralMode === LateralMode.RWY_TRACK) {
        text = 'RWY TRK';
        id = 7;
    } else if (activeLateralMode === LateralMode.TRACK) {
        text = 'TRACK';
        id = 8;
    } else if (activeLateralMode === LateralMode.LOC_TRACK) {
        text = 'LOC';
        id = 10;
    } else if (activeLateralMode === LateralMode.NAV && !finalArmed && activeVerticalMode !== VerticalMode.FINAL) {
        text = 'NAV';
        id = 13;
    } else if (activeLateralMode === LateralMode.NAV && finalArmed && activeVerticalMode !== VerticalMode.FINAL) {
        text = 'APP NAV';
        id = 12;
    } else {
        return null;
    }
    // case 2:
    //     text = 'LOC B/C*';
    //     id = 2;
    //     break;
    // case 4:
    //     text = 'F-LOC*';
    //     id = 4;
    //     break;
    // case 9:
    //     text = 'LOC B/C';
    //     id = 9;
    //     break;
    // case 11:
    //     text = 'F-LOC';
    //     id = 11;
    //     break;

    return (
        <g>
            <ShowForSeconds timer={10} id={id}>
                <path className="NormalStroke White" d="m100.87 1.8143v6.0476h-33.075l1e-6 -6.0476z" />
            </ShowForSeconds>
            <text className="FontMedium MiddleAlign Green" x="84.856567" y="6.9873109">{text}</text>
        </g>
    );
};

const C2Cell = () => {
    const armedLateralBitmask = getSimVar('L:A32NX_FMA_LATERAL_ARMED', 'number');

    const navArmed = isArmed(armedLateralBitmask, ArmedLateralMode.NAV);
    const locArmed = isArmed(armedLateralBitmask, ArmedLateralMode.LOC);

    const armedVerticalBitmask = getSimVar('L:A32NX_FMA_VERTICAL_ARMED', 'number');
    const finalArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.FINAL);

    const activeVerticalMode = getSimVar('L:A32NX_FMA_VERTICAL_MODE', 'enum');

    let text: string;
    if (locArmed) {
        // case 1:
        //     text = 'LOC B/C';
        //     break;
        text = 'LOC';
        // case 3:
        //     text = 'F-LOC';
        //     break;
    } else if (navArmed && (finalArmed || activeVerticalMode === VerticalMode.FINAL)) {
        text = 'APP NAV';
    } else if (navArmed) {
        text = 'NAV';
    } else {
        return null;
    }

    return (
        <text className="FontMedium MiddleAlign Cyan" x="84.734184" y="14.440415">{text}</text>
    );
};

const BC1Cell = () => {
    const activeVerticalMode = getSimVar('L:A32NX_FMA_VERTICAL_MODE', 'enum');
    const activeLateralMode = getSimVar('L:A32NX_FMA_LATERAL_MODE', 'number');

    let text: string;
    let id = 0;
    if (activeVerticalMode === VerticalMode.ROLL_OUT) {
        text = 'ROLL OUT';
        id = 1;
    } else if (activeVerticalMode === VerticalMode.FLARE) {
        text = 'FLARE';
        id = 2;
    } else if (activeVerticalMode === VerticalMode.LAND) {
        text = 'LAND';
        id = 3;
    } else if (activeVerticalMode === VerticalMode.FINAL && activeLateralMode === LateralMode.NAV) {
        text = 'FINAL APP';
        id = 4;
    } else {
        return null;
    }

    return (
        <g>
            <ShowForSeconds timer={9} id={id}>
                <path className="NormalStroke White" d="m50.178 1.8143h35.174v6.0476h-35.174z" />
            </ShowForSeconds>
            <text className="FontMedium MiddleAlign Green" x="67.923119" y="7.0302663">{text}</text>
        </g>
    );
};

const getBC3Message = (isAttExcessive: boolean) => {
    const armedVerticalBitmask = getSimVar('L:A32NX_FMA_VERTICAL_ARMED', 'number');
    const TCASArmed = isArmed(armedVerticalBitmask, ArmedVerticalMode.TCAS);

    const trkFpaDeselectedTCAS = getSimVar('L:A32NX_AUTOPILOT_TCAS_MESSAGE_TRK_FPA_DESELECTION', 'bool');
    const tcasRaInhibited = getSimVar('L:A32NX_AUTOPILOT_TCAS_MESSAGE_RA_INHIBITED', 'bool');

    const setHoldSpeed = getSimVar('L:A32NX_PFD_MSG_SET_HOLD_SPEED', 'bool');

    let text: string;
    let className: string;
    // All currently unused message are set to false
    if (false) {
        text = 'MAN PITCH TRIM ONLY';
        className = 'Red Blink9Seconds';
    } else if (false) {
        text = 'USE MAN PITCH TRIM';
        className = 'PulseAmber9Seconds Amber';
    } else if (false) {
        text = 'FOR GA: SET TOGA';
        className = 'PulseAmber9Seconds Amber';
    } else if (TCASArmed && !isAttExcessive) {
        text = '  TCAS                ';
        className = 'Cyan';
    } else if (false) {
        text = 'DISCONNECT AP FOR LDG';
        className = 'PulseAmber9Seconds Amber';
    } else if (tcasRaInhibited && !isAttExcessive) {
        text = 'TCAS RA INHIBITED';
        className = 'White';
    } else if (trkFpaDeselectedTCAS && !isAttExcessive) {
        text = 'TRK FPA DESELECTED';
        className = 'White';
    } else if (false) {
        text = 'SET GREEN DOT SPEED';
        className = 'White';
    } else if (false) {
        text = 'T/D REACHED';
        className = 'White';
    } else if (false) {
        text = 'MORE DRAG';
        className = 'White';
    } else if (false) {
        text = 'CHECK SPEED MODE';
        className = 'White';
    } else if (false) {
        text = 'CHECK APPR SELECTION';
        className = 'White';
    } else if (false) {
        text = 'TURN AREA EXCEEDANCE';
        className = 'White';
    } else if (setHoldSpeed) {
        text = 'SET HOLD SPEED';
        className = 'White';
    } else if (false) {
        text = 'VERT DISCONT AHEAD';
        className = 'Amber';
    } else if (false) {
        text = 'FINAL APP SELECTED';
        className = 'White';
    } else {
        return [null, null];
    }

    return [text, className];
};

const BC3Cell = ({ isAttExcessive }) => {
    const [text, className] = getBC3Message(isAttExcessive);

    if (text !== null) {
        return (
            <text className={`FontMedium MiddleAlign ${className}`} x="68.087875" y="21.627102" xmlSpace="preserve">{text}</text>
        );
    }
    return null;
};

const D1D2Cell = () => {
    const approachCapability = getSimVar('L:A32NX_ApproachCapability', 'enum');

    let text1: string;
    let text2: string | undefined;
    switch (approachCapability) {
    case 1:
        text1 = 'CAT1';
        break;
    case 2:
        text1 = 'CAT2';
        break;
    case 3:
        text1 = 'CAT3';
        text2 = 'SINGLE';
        break;
    case 4:
        text1 = 'CAT3';
        text2 = 'DUAL';
        break;
    case 5:
        text1 = 'AUTO';
        text2 = 'LAND';
        break;
    case 6:
        text1 = 'F-APP';
        break;
    case 7:
        text1 = 'F-APP';
        text2 = '+ RAW';
        break;
    case 8:
        text1 = 'RAW';
        text2 = 'ONLY';
        break;
    default:
        return null;
    }

    const box = text2 ? <path className="NormalStroke White" d="m104.1 1.8143h27.994v13.506h-27.994z" />
        : <path className="NormalStroke White" d="m104.1 1.8143h27.994v6.0476h-27.994z" />;

    return (
        <g>
            <text className="FontMedium MiddleAlign White" x="118.45866" y="7.125926">{text1}</text>
            {text2
            && <text className="FontMedium MiddleAlign White" x="118.39752" y="14.289783">{text2}</text>}
            <ShowForSeconds id={approachCapability} timer={9}>
                {box}
            </ShowForSeconds>
        </g>
    );
};

const D3Cell = () => {
    const MDA = getSimVar('L:AIRLINER_MINIMUM_DESCENT_ALTITUDE', 'feet');
    let text: JSX.Element | string | null = null;
    let fontSize = 'FontSmallest';
    if (MDA !== 0) {
        const MDAText = Math.round(MDA).toString().padStart(6, ' ');
        text = (
            <>
                <tspan>BARO</tspan>
                <tspan className="Cyan" xmlSpace="preserve">{MDAText}</tspan>
            </>
        );
    } else {
        const DH = getSimVar('L:AIRLINER_DECISION_HEIGHT', 'feet');
        if (DH !== -1 && DH !== -2) {
            const DHText = Math.round(DH).toString().padStart(4, ' ');
            text = (
                <>
                    <tspan>RADIO</tspan>
                    <tspan className="Cyan" xmlSpace="preserve">{DHText}</tspan>
                </>
            );
        } else if (DH === -2) {
            text = 'NO DH';
            fontSize = 'FontMedium';
        }
    }

    return (
        <text className={`${fontSize} MiddleAlign White`} x="118.38384" y="21.104172">{text}</text>
    );
};

const E1Cell = () => {
    const AP1Engaged = getSimVar('L:A32NX_AUTOPILOT_1_ACTIVE', 'bool');
    const AP2Engaged = getSimVar('L:A32NX_AUTOPILOT_2_ACTIVE', 'bool');

    if (!AP1Engaged && !AP2Engaged) {
        return null;
    }

    let text: string;
    let id = 0;
    if (AP1Engaged && !AP2Engaged) {
        text = 'AP1';
        id = 1;
    } else if (AP2Engaged && !AP1Engaged) {
        text = 'AP2';
        id = 2;
    } else {
        text = 'AP1+2';
        id = 3;
    }

    return (
        <g>
            <ShowForSeconds timer={9} id={id}>
                <path className="NormalStroke White" d="m156.13 1.8143v6.0476h-20.81v-6.0476z" />
            </ShowForSeconds>
            <text className="FontMedium MiddleAlign White" x="145.95334" y="7.1154728">{text}</text>
        </g>
    );
};

const E2Cell = () => {
    const FD1Active = getSimVar('AUTOPILOT FLIGHT DIRECTOR ACTIVE:1', 'bool');
    const FD2Active = getSimVar('AUTOPILOT FLIGHT DIRECTOR ACTIVE:2', 'bool');

    if (!FD1Active && !FD2Active && !getSimVar('L:A32NX_AUTOPILOT_1_ACTIVE', 'bool') && !getSimVar('L:A32NX_AUTOPILOT_2_ACTIVE', 'bool')) {
        return null;
    }

    let leftString;
    let id = 0;
    if (FD1Active) {
        leftString = '1';
        id |= (1 << 0);
    } else {
        leftString = '-';
        id |= (1 << 1);
    }
    let rightString;
    if (FD2Active) {
        rightString = '2';
        id |= (1 << 2);
    } else {
        rightString = '-';
        id |= (1 << 3);
    }

    const text = `${leftString} FD ${rightString}`;
    return (
        <g>
            <ShowForSeconds timer={9} id={id}>
                <path d="m156.13 9.0715v6.0476h-20.81v-6.0476z" className="NormalStroke White" />
            </ShowForSeconds>
            <text className="FontMedium MiddleAlign White" style={{ wordSpacing: '-1.9844px' }} x="145.95045" y="14.417698">{text}</text>
        </g>
    );
};

const E3Cell = () => {
    const status = getSimVar('L:A32NX_AUTOTHRUST_STATUS', 'enum');

    let style: string;
    let yPos: number;
    let id = 0;
    switch (status) {
    case 1:
        style = 'FontSmall Cyan';
        yPos = 21.253048;
        id = 1;
        break;
    case 2:
        style = 'FontMedium White';
        yPos = 21.753487;
        id = 2;
        break;
    default:
        return null;
    }

    return (
        <g>
            <ShowForSeconds timer={9} id={id}>
                <path className="NormalStroke White" d="m135.32 16.329h20.81v6.0476h-20.81z" />
            </ShowForSeconds>
            <text className={`MiddleAlign ${style}`} x="145.87538" y={yPos}>A/THR</text>
        </g>
    );
};

interface ShowForSecondsProps {
    timer: number;
    id: number;
}

class ShowForSeconds extends Component<ShowForSecondsProps> {
    private Timer: number;

    private PrevID: number;

    private GetDeltaTime: () => number;

    private Update: () => void;

    constructor(props: ShowForSecondsProps) {
        super(props);

        this.Timer = this.props.timer || 10;
        this.PrevID = NaN;

        this.GetDeltaTime = createDeltaTimeCalculator();
        const updateFunction = () => {
            const deltaTime = this.GetDeltaTime();
            if (this.Timer > 0) {
                this.Timer -= deltaTime / 1000;
            }
            this.forceUpdate();
        };
        this.Update = updateFunction.bind(this);
    }

    componentDidMount() {
        renderTarget.parentElement.addEventListener('update', this.Update);
    }

    shouldComponentUpdate(nextProps: ShowForSecondsProps) {
        if (this.PrevID !== nextProps.id) {
            this.PrevID = nextProps.id;
            this.Timer = nextProps.timer || 10;
        }

        return false;
    }

    componentWillUnmount() {
        renderTarget.parentElement.removeEventListener('update', this.Update);
    }

    render() {
        if (this.Timer > 0) {
            return (
                this.props.children
            );
        }
        return null;
    }
}
