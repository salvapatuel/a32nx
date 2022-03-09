import { VerticalProfileComputationParametersObserver } from '@fmgc/guidance/vnav/VerticalProfileComputationParameters';
import { Constants } from '@shared/Constants';
import { Step, StepCoordinator } from '@fmgc/guidance/vnav/StepCoordinator';
import { VnavConfig } from '@fmgc/guidance/vnav/VnavConfig';
import { ClimbStrategy } from '@fmgc/guidance/vnav/climb/ClimbStrategy';
import { DescentStrategy } from '@fmgc/guidance/vnav/descent/DescentStrategy';
import { BaseGeometryProfile } from '@fmgc/guidance/vnav/profile/BaseGeometryProfile';
import { ManagedSpeedType, SpeedProfile } from '@fmgc/guidance/vnav/climb/SpeedProfile';
import { EngineModel } from '@fmgc/guidance/vnav/EngineModel';
import { WindComponent } from '@fmgc/guidance/vnav/wind';
import { Predictions, StepResults } from '../Predictions';
import { MaxSpeedConstraint, VerticalCheckpoint, VerticalCheckpointReason } from '../profile/NavGeometryProfile';
import { AtmosphericConditions } from '../AtmosphericConditions';

export interface CruisePathBuilderResults {
    remainingFuelOnBoardAtTopOfDescent: number,
    secondsFromPresentAtTopOfDescent: number
}

export class CruisePathBuilder {
    constructor(private computationParametersObserver: VerticalProfileComputationParametersObserver,
        private atmosphericConditions: AtmosphericConditions,
        private stepCoordinator: StepCoordinator) { }

    update() {
        this.atmosphericConditions.update();
    }

    computeCruisePath(profile: BaseGeometryProfile, stepClimbStrategy: ClimbStrategy, stepDescentStrategy: DescentStrategy, speedProfile: SpeedProfile): CruisePathBuilderResults {
        const topOfClimb = profile.findVerticalCheckpoint(VerticalCheckpointReason.TopOfClimb);
        const presentPosition = profile.findVerticalCheckpoint(VerticalCheckpointReason.PresentPosition);

        const startOfCruise = topOfClimb ?? presentPosition;

        const topOfDescent = profile.findVerticalCheckpoint(VerticalCheckpointReason.TopOfDescent);

        if (!startOfCruise?.distanceFromStart || !topOfDescent?.distanceFromStart) {
            throw new Error('[FMS/VNAV] Start of cruise or T/D could not be located');
        }

        if (startOfCruise.distanceFromStart > topOfDescent.distanceFromStart) {
            throw new Error('[FMS/VNAV] Cruise segment too short');
        }

        const checkpointsToAdd: VerticalCheckpoint[] = [startOfCruise];

        for (const step of this.stepCoordinator.steps) {
            // If the step is too close to T/D
            if (step.isIgnored) {
                continue;
            }

            if (step.distanceFromStart < startOfCruise.distanceFromStart || step.distanceFromStart > topOfDescent.distanceFromStart) {
                if (VnavConfig.DEBUG_PROFILE) {
                    console.warn(
                        `[FMS/VNAV] Cruise step is not within cruise segment \
                        (${step.distanceFromStart.toFixed(2)} NM, T/C: ${startOfCruise.distanceFromStart.toFixed(2)} NM, T/D: ${topOfDescent.distanceFromStart.toFixed(2)} NM)`,
                    );
                }

                continue;
            }

            // See if there are any speed constraints before the step
            for (const speedConstraint of profile.maxClimbSpeedConstraints) {
                if (speedConstraint.distanceFromStart > step.distanceFromStart) {
                    continue;
                }

                this.addSegmentToSpeedConstraint(checkpointsToAdd, speedConstraint, speedProfile);
            }

            const { distanceFromStart, altitude, remainingFuelOnBoard } = checkpointsToAdd[checkpointsToAdd.length - 1];

            const speed = speedProfile.getTarget(distanceFromStart, altitude, ManagedSpeedType.Cruise);
            const segmentToStep = this.computeCruiseSegment(step.distanceFromStart - distanceFromStart, remainingFuelOnBoard, speed);
            this.addNewCheckpointFromResult(checkpointsToAdd, segmentToStep, VerticalCheckpointReason.AtmosphericConditions);

            this.addStepFromLastCheckpoint(checkpointsToAdd, step, stepClimbStrategy, stepDescentStrategy);
        }

        // Once again, we check if there are any speed constraints before the T/D
        for (const speedConstraint of profile.maxClimbSpeedConstraints) {
            // If speed constraint does not lie along the remaining cruise track
            if (speedConstraint.distanceFromStart > topOfDescent.distanceFromStart) {
                continue;
            }

            this.addSegmentToSpeedConstraint(checkpointsToAdd, speedConstraint, speedProfile);
        }

        const speedTarget = speedProfile.getTarget(
            checkpointsToAdd[checkpointsToAdd.length - 1].distanceFromStart,
            checkpointsToAdd[checkpointsToAdd.length - 1].altitude,
            ManagedSpeedType.Cruise,
        );

        if (speedTarget - 0 > 1) {
            const accelerationStep = this.levelAccelerationStep(
                startOfCruise.remainingFuelOnBoard,
                checkpointsToAdd[checkpointsToAdd.length - 1].speed,
                speedTarget,
            );

            this.addNewCheckpointFromResult(checkpointsToAdd, accelerationStep, VerticalCheckpointReason.AtmosphericConditions);
        }

        if (topOfDescent.distanceFromStart < checkpointsToAdd[checkpointsToAdd.length - 1].distanceFromStart) {
            console.warn('[FMS/VNAV] An acceleration step in the cruise took us past T/D. This is not implemented properly yet. Blame BBK');
        }

        const { fuelBurned, timeElapsed } = this.computeCruiseSegment(
            topOfDescent.distanceFromStart - checkpointsToAdd[checkpointsToAdd.length - 1].distanceFromStart,
            startOfCruise.remainingFuelOnBoard,
            speedTarget,
        );

        if (checkpointsToAdd.length > 0) {
            profile.addCheckpointAtDistanceFromStart(checkpointsToAdd[1].distanceFromStart, ...checkpointsToAdd.slice(1));
        }

        return {
            remainingFuelOnBoardAtTopOfDescent: checkpointsToAdd[checkpointsToAdd.length - 1].remainingFuelOnBoard - fuelBurned,
            secondsFromPresentAtTopOfDescent: checkpointsToAdd[checkpointsToAdd.length - 1].secondsFromPresent + timeElapsed,
        };
    }

    private addSegmentToSpeedConstraint(checkpoints: VerticalCheckpoint[], speedConstraint: MaxSpeedConstraint, speedProfile: SpeedProfile) {
        const { distanceFromStart, altitude, remainingFuelOnBoard } = checkpoints[checkpoints.length - 1];

        if (speedConstraint.distanceFromStart < distanceFromStart) {
            return;
        }

        const speed = speedProfile.getTarget(distanceFromStart, altitude, ManagedSpeedType.Cruise);
        const segmentResult = this.computeCruiseSegment(
            speedConstraint.distanceFromStart - distanceFromStart,
            remainingFuelOnBoard,
            speed,
        );

        this.addNewCheckpointFromResult(checkpoints, segmentResult, VerticalCheckpointReason.SpeedConstraint);
    }

    private addStepFromLastCheckpoint(checkpoints: VerticalCheckpoint[], step: Step, stepClimbStrategy: ClimbStrategy, stepDescentStrategy: DescentStrategy) {
        // TODO: What happens if the step is at cruise altitude?
        const { managedCruiseSpeed, managedCruiseSpeedMach } = this.computationParametersObserver.get();
        const { altitude, remainingFuelOnBoard } = checkpoints[checkpoints.length - 1];

        const isClimbVsDescent = step.toAltitude > altitude;
        // Instead of just atmospheric conditions, the last checkpoint is now a step climb point
        if (checkpoints[checkpoints.length - 1].reason === VerticalCheckpointReason.AtmosphericConditions) {
            checkpoints[checkpoints.length - 1].reason = isClimbVsDescent
                ? VerticalCheckpointReason.StepClimb
                : VerticalCheckpointReason.StepDescent;
        }

        const stepResults = isClimbVsDescent
            ? stepClimbStrategy.predictToAltitude(altitude, step.toAltitude, managedCruiseSpeed, managedCruiseSpeedMach, remainingFuelOnBoard, WindComponent.zero())
            : stepDescentStrategy.predictToAltitude(altitude, step.toAltitude, managedCruiseSpeed, managedCruiseSpeed, remainingFuelOnBoard, WindComponent.zero());

        this.addNewCheckpointFromResult(checkpoints, stepResults, isClimbVsDescent ? VerticalCheckpointReason.TopOfStepClimb : VerticalCheckpointReason.BottomOfStepDescent);
    }

    private computeCruiseSegment(distance: NauticalMiles, remainingFuelOnBoard: number, speed: Knots): StepResults {
        const { zeroFuelWeight, cruiseAltitude, managedCruiseSpeedMach } = this.computationParametersObserver.get();

        return Predictions.levelFlightStep(
            cruiseAltitude,
            distance,
            speed,
            managedCruiseSpeedMach,
            zeroFuelWeight * Constants.TONS_TO_POUNDS,
            remainingFuelOnBoard,
            0,
            this.atmosphericConditions.isaDeviation,
        );
    }

    private levelAccelerationStep(remainingFuelOnBoard: number, speed: Knots, finalSpeed: Knots): StepResults {
        const { zeroFuelWeight, cruiseAltitude, managedCruiseSpeedMach, tropoPause } = this.computationParametersObserver.get();

        return Predictions.speedChangeStep(
            0,
            cruiseAltitude,
            speed,
            finalSpeed,
            managedCruiseSpeedMach,
            managedCruiseSpeedMach,
            this.getClimbThrustN1Limit(this.atmosphericConditions, cruiseAltitude, speed),
            zeroFuelWeight * Constants.TONS_TO_POUNDS,
            remainingFuelOnBoard,
            0,
            this.atmosphericConditions.isaDeviation,
            tropoPause,
        );
    }

    getFinalCruiseAltitude(): Feet {
        const { cruiseAltitude } = this.computationParametersObserver.get();

        if (this.stepCoordinator.steps.length === 0) {
            return cruiseAltitude;
        }

        return this.stepCoordinator.steps[this.stepCoordinator.steps.length - 1].toAltitude;
    }

    private addNewCheckpointFromResult(existingCheckpoints: VerticalCheckpoint[], result: StepResults, reason: VerticalCheckpointReason) {
        const { distanceFromStart, secondsFromPresent, remainingFuelOnBoard } = existingCheckpoints[existingCheckpoints.length - 1];

        existingCheckpoints.push({
            reason,
            distanceFromStart: distanceFromStart + result.distanceTraveled,
            altitude: result.finalAltitude,
            secondsFromPresent: secondsFromPresent + result.timeElapsed,
            speed: result.speed,
            remainingFuelOnBoard: remainingFuelOnBoard - result.fuelBurned,
        });
    }

    private getClimbThrustN1Limit(atmosphericConditions: AtmosphericConditions, altitude: Feet, speed: Knots) {
        // This Mach number is the Mach number for the predicted climb speed, not the Mach to use after crossover altitude.
        const climbSpeedMach = atmosphericConditions.computeMachFromCas(altitude, speed);
        const estimatedTat = atmosphericConditions.totalAirTemperatureFromMach(altitude, climbSpeedMach);

        return EngineModel.tableInterpolation(EngineModel.maxClimbThrustTableLeap, estimatedTat, altitude);
    }
}
