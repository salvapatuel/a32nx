export class SpeedMargin {
    private vmo: Knots = 350;

    private mmo: Mach = 0.82;

    constructor(private managedDescentSpeed: Knots) { }

    getTarget(indicatedAirspeed: Knots, targetSpeed: Knots): Knots {
        const [lowerMargin, upperMargin] = this.getMargins(targetSpeed);

        return Math.max(Math.min(indicatedAirspeed, upperMargin), lowerMargin);
    }

    getMargins(currentTarget: Knots): [Knots, Knots] {
        const vmax = SimVar.GetSimVarValue('L:A32NX_SPEEDS_VMAX', 'number');
        const vls = SimVar.GetSimVarValue('L:A32NX_SPEEDS_VLS', 'number');

        const mmoAsIas = SimVar.GetGameVarValue('FROM MACH TO KIAS', 'number', this.mmo);

        const distanceToUpperMargin = this.managedDescentSpeed - currentTarget > 1 ? 5 : 20;

        return [
            Math.max(vls, currentTarget - 20),
            Math.min(vmax, this.vmo - 3, mmoAsIas - 0.006, currentTarget + distanceToUpperMargin),
        ];
    }
}
