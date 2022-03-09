//  Copyright (c) 2022 FlyByWire Simulations
//  SPDX-License-Identifier: GPL-3.0

import { Common } from './common';

export class AtmosphericConditions {
    private ambientTemperatureFromSim: Celcius;

    private altitudeFromSim: Feet;

    private casFromSim: Knots;

    private computedIsaDeviation: Celcius;

    constructor(private tropoPause: Feet) {
        this.update();
    }

    update() {
        this.ambientTemperatureFromSim = SimVar.GetSimVarValue('AMBIENT TEMPERATURE', 'celsius');
        this.altitudeFromSim = SimVar.GetSimVarValue('INDICATED ALTITUDE', 'feet');
        this.casFromSim = this.computeCasFromTas(this.altitudeFromSim, SimVar.GetSimVarValue('AIRSPEED TRUE', 'knots'));

        this.computedIsaDeviation = this.ambientTemperatureFromSim - Common.getIsaTemp(this.altitudeFromSim);
    }

    get currentStaticAirTemperature(): Celcius {
        return this.ambientTemperatureFromSim;
    }

    get currentAltitude(): Feet {
        return this.altitudeFromSim;
    }

    get currentAirspeed(): Knots {
        return this.casFromSim;
    }

    get isaDeviation(): Celcius {
        return this.computedIsaDeviation;
    }

    predictStaticAirTemperatureAtAltitude(altitude: Feet): number {
        return Common.getIsaTemp(altitude, altitude > this.tropoPause) + this.isaDeviation;
    }

    totalAirTemperatureFromMach(altitude: Feet, mach: number) {
        // From https://en.wikipedia.org/wiki/Total_air_temperature, using gamma = 1.4
        return (this.predictStaticAirTemperatureAtAltitude(altitude) + 273.15) * (1 + 0.2 * mach ** 2) - 273.15;
    }

    computeMachFromCas(altitude: Feet, speed: Knots): number {
        const deltaSrs = Common.getDelta(altitude, altitude > this.tropoPause);

        return Common.CAStoMach(speed, deltaSrs);
    }

    computeCasFromMach(altitude: Feet, mach: Mach): number {
        const deltaSrs = Common.getDelta(altitude, altitude > this.tropoPause);

        return Common.machToCas(mach, deltaSrs);
    }

    computeCasFromTas(altitude: Feet, speed: Knots): Knots {
        const thetaSrs = Common.getTheta(altitude, this.isaDeviation, altitude > this.tropoPause);
        const deltaSrs = Common.getDelta(altitude, altitude > this.tropoPause);

        return Common.TAStoCAS(speed, thetaSrs, deltaSrs);
    }

    computeTasFromCas(altitude: Feet, speed: Knots): Knots {
        const thetaSrs = Common.getTheta(altitude, this.isaDeviation, altitude > this.tropoPause);
        const deltaSrs = Common.getDelta(altitude, altitude > this.tropoPause);

        return Common.CAStoTAS(speed, thetaSrs, deltaSrs);
    }
}
