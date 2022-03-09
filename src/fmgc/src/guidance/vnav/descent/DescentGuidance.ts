import { RequestedVerticalMode, TargetAltitude, TargetVerticalSpeed } from '@fmgc/guidance/ControlLaws';
import { AtmosphericConditions } from '@fmgc/guidance/vnav/AtmosphericConditions';
import { AircraftToDescentProfileRelation } from '@fmgc/guidance/vnav/descent/AircraftToProfileRelation';
import { NavGeometryProfile } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { VerticalMode } from '@shared/autopilot';
import { FmgcFlightPhase } from '@shared/flightphase';
import { SpeedMargin } from './SpeedMargin';

enum DescentGuidanceState {
    InvalidProfile,
    ProvidingGuidance,
    Observing
}

export class DescentGuidance {
    private state: DescentGuidanceState = DescentGuidanceState.InvalidProfile;

    private requestedVerticalMode: RequestedVerticalMode = RequestedVerticalMode.None;

    private targetAltitude: TargetAltitude = 0;

    private targetAltitudeGuidance: TargetAltitude = 0;

    private targetVerticalSpeed: TargetVerticalSpeed = 0;

    private showLinearDeviationOnPfd: boolean = false;

    private showDescentLatchOnPfd: boolean = false;

    private speedMargin: SpeedMargin;

    private speedTarget: Knots | Mach;

    constructor(
        private aircraftToDescentProfileRelation: AircraftToDescentProfileRelation,
        private observer: VerticalProfileComputationParametersObserver,
        private atmosphericConditions: AtmosphericConditions,
    ) {
        const { managedDescentSpeed } = this.observer.get();
        this.speedMargin = new SpeedMargin(managedDescentSpeed);

        this.writeToSimVars();
    }

    updateProfile(profile: NavGeometryProfile) {
        this.aircraftToDescentProfileRelation.updateProfile(profile);

        if (!this.aircraftToDescentProfileRelation.isValid) {
            this.changeState(DescentGuidanceState.InvalidProfile);
            return;
        }

        this.changeState(DescentGuidanceState.Observing);
    }

    private changeState(newState: DescentGuidanceState) {
        if (this.state === newState) {
            return;
        }

        if (this.state !== DescentGuidanceState.InvalidProfile && newState === DescentGuidanceState.InvalidProfile) {
            this.reset();
            this.writeToSimVars();
        }

        this.state = newState;
    }

    private reset() {
        this.requestedVerticalMode = RequestedVerticalMode.None;
        this.targetAltitude = 0;
        this.targetVerticalSpeed = 0;
        this.showLinearDeviationOnPfd = false;
        this.showDescentLatchOnPfd = false;
    }

    update() {
        this.aircraftToDescentProfileRelation.update();

        if (!this.aircraftToDescentProfileRelation.isValid) {
            return;
        }

        if (this.observer.get().fcuVerticalMode === VerticalMode.DES) {
            this.changeState(DescentGuidanceState.ProvidingGuidance);
        }

        this.updateSpeedTarget();
        this.updateLinearDeviation();

        if (this.state === DescentGuidanceState.ProvidingGuidance) {
            this.updateDesModeGuidance();
        }

        this.writeToSimVars();
    }

    private updateLinearDeviation() {
        this.targetAltitude = this.aircraftToDescentProfileRelation.currentTargetAltitude();

        this.showLinearDeviationOnPfd = this.observer.get().flightPhase >= FmgcFlightPhase.Descent || this.aircraftToDescentProfileRelation.isPastTopOfDescent();
    }

    private updateDesModeGuidance() {
        const isOnGeometricPath = this.aircraftToDescentProfileRelation.isOnGeometricPath();
        const isAboveSpeedLimitAltitude = this.aircraftToDescentProfileRelation.isAboveSpeedLimitAltitude();
        const isBeforeTopOfDescent = !this.aircraftToDescentProfileRelation.isPastTopOfDescent();
        const linearDeviation = this.aircraftToDescentProfileRelation.computeLinearDeviation();

        // const airspeed = SimVar.GetSimVarValue('AIRSPEED INDICATED', 'Knots');
        // SimVar.SetSimVarValue('L:A32NX_SPEEDS_MANAGED_ATHR', 'knots', this.speedMargin.getTarget(airspeed));

        this.targetAltitudeGuidance = this.atmosphericConditions.estimatePressureAltitudeInMsfs(
            this.aircraftToDescentProfileRelation.currentTargetAltitude(),
        );

        if (isBeforeTopOfDescent || linearDeviation < -200) {
            // below path
            if (isOnGeometricPath) {
                this.requestedVerticalMode = RequestedVerticalMode.FpaSpeed;
                this.targetVerticalSpeed = this.aircraftToDescentProfileRelation.currentTargetPathAngle() / 2;
            } else {
                this.requestedVerticalMode = RequestedVerticalMode.VsSpeed;
                this.targetVerticalSpeed = (isAboveSpeedLimitAltitude ? -1000 : -500);
            }
        } else if (linearDeviation > 200) {
            // above path
            this.requestedVerticalMode = RequestedVerticalMode.SpeedThrust;
        } else if (isOnGeometricPath) {
            // on geometric path

            this.requestedVerticalMode = RequestedVerticalMode.VpathSpeed;
            this.targetVerticalSpeed = this.aircraftToDescentProfileRelation.currentTargetVerticalSpeed();
        } else {
            // on idle path

            this.requestedVerticalMode = RequestedVerticalMode.VpathThrust;
            this.targetVerticalSpeed = this.aircraftToDescentProfileRelation.currentTargetVerticalSpeed();
        }
    }

    private updateSpeedTarget() {
        const { fcuSpeed } = this.observer.get();

        this.speedTarget = fcuSpeed > 0
            ? fcuSpeed
            : Math.round(this.aircraftToDescentProfileRelation.currentTargetSpeed());
    }

    private writeToSimVars() {
        SimVar.SetSimVarValue('L:A32NX_FG_REQUESTED_VERTICAL_MODE', 'Enum', this.requestedVerticalMode);
        SimVar.SetSimVarValue('L:A32NX_FG_TARGET_ALTITUDE', 'Feet', this.targetAltitudeGuidance);
        SimVar.SetSimVarValue('L:A32NX_FG_TARGET_VERTICAL_SPEED', 'number', this.targetVerticalSpeed);

        SimVar.SetSimVarValue('L:A32NX_PFD_TARGET_ALTITUDE', 'Feet', this.targetAltitude);
        SimVar.SetSimVarValue('L:A32NX_PFD_LINEAR_DEVIATION_ACTIVE', 'Bool', this.showLinearDeviationOnPfd);
        SimVar.SetSimVarValue('L:A32NX_PFD_VERTICAL_PROFILE_LATCHED', 'Bool', this.showDescentLatchOnPfd);

        const [lower, upper] = this.speedMargin.getMargins(this.speedTarget);

        SimVar.SetSimVarValue('L:A32NX_PFD_LOWER_SPEED_MARGIN', 'Knots', lower);
        SimVar.SetSimVarValue('L:A32NX_PFD_UPPER_SPEED_MARGIN', 'Knots', upper);
    }
}
