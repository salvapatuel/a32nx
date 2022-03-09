import { InertialDistanceAlongTrack } from '@fmgc/guidance/vnav/descent/InertialDistanceAlongTrack';
import { NavGeometryProfile, VerticalCheckpoint, VerticalCheckpointReason } from '@fmgc/guidance/vnav/profile/NavGeometryProfile';
import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { MathUtils } from '@shared/MathUtils';

export class AircraftToDescentProfileRelation {
    public isValid: boolean = false;

    private currentProfile?: NavGeometryProfile;

    private inertialDistanceAlongTrack: InertialDistanceAlongTrack;

    private topOfDescent?: VerticalCheckpoint;

    private geometricPathStart?: VerticalCheckpoint;

    constructor(private observer: VerticalProfileComputationParametersObserver) {
        this.inertialDistanceAlongTrack = new InertialDistanceAlongTrack();
    }

    updateProfile(profile: NavGeometryProfile) {
        const topOfDescent = profile?.findVerticalCheckpoint(VerticalCheckpointReason.TopOfDescent);
        const lastPosition = profile?.findVerticalCheckpoint(VerticalCheckpointReason.PresentPosition);
        const geometricPathStart = profile?.findVerticalCheckpoint(VerticalCheckpointReason.GeometricPathStart);

        const isProfileValid = !!topOfDescent && !!lastPosition && !!geometricPathStart;

        if (!isProfileValid) {
            this.invalidate();
            return;
        }

        this.isValid = isProfileValid;

        this.topOfDescent = topOfDescent;
        this.geometricPathStart = geometricPathStart;

        // TODO: Remove this
        profile.checkpoints = profile.checkpoints.filter(({ reason }) => reason !== VerticalCheckpointReason.PresentPosition);

        if (VnavConfig.DEBUG_PROFILE && this.currentProfile) {
            // How much the distance to the end of the path changed between the current profile and the new one.
            // Ideally, this should be as low as possible. Otherwise, there might be a bug
            const distanceToEndDeviation = profile.getDistanceFromStart(this.inertialDistanceAlongTrack.get())
                - this.currentProfile.getDistanceFromStart(this.currentProfile.distanceToPresentPosition);
            console.log(`[FMS/VNAV] distanceToEndDeviation: ${distanceToEndDeviation}`);
        }

        this.currentProfile = profile;

        this.inertialDistanceAlongTrack.updateCorrectInformation(lastPosition.distanceFromStart);
    }

    private invalidate() {
        this.isValid = false;
        this.currentProfile = undefined;
        this.topOfDescent = undefined;
    }

    update() {
        if (!this.isValid) {
            return;
        }

        this.inertialDistanceAlongTrack.update();
    }

    isPastTopOfDescent(): boolean {
        return this.inertialDistanceAlongTrack.get() > this.topOfDescent.distanceFromStart;
    }

    isOnGeometricPath(): boolean {
        return this.inertialDistanceAlongTrack.get() > this.geometricPathStart.distanceFromStart;
    }

    computeLinearDeviation(): Feet {
        const altitude = this.observer.get().presentPosition.alt;
        const targetAltitude = this.currentTargetAltitude();

        return altitude - targetAltitude;
    }

    currentTargetAltitude(): Feet {
        return this.currentProfile.interpolateAltitudeAtDistance(this.inertialDistanceAlongTrack.get());
    }

    currentTargetPathAngle(): Degrees {
        return this.currentProfile.interpolatePathAngleAtDistance(this.inertialDistanceAlongTrack.get());
    }

    currentTargetVerticalSpeed(): FeetPerMinute {
        const groundSpeed = SimVar.GetSimVarValue('GPS GROUND SPEED', 'Knots');

        const knotsToFeetPerMinute = 101.269;
        return knotsToFeetPerMinute * groundSpeed * Math.tan(this.currentTargetPathAngle() * MathUtils.DEGREES_TO_RADIANS);
    }

    isAboveSpeedLimitAltitude(): boolean {
        const { presentPosition, descentSpeedLimit } = this.observer.get();

        return presentPosition.alt > descentSpeedLimit?.underAltitude;
    }
}
