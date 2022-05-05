function suitability = isSetupSuitable()
    % Determine whether the given motor (A,B or C) and setup (RWD or AWD)
    % can run the vehicle at 130kmph at 12% grade

    global Cd fArea airDen fr mVehicle g gearRatio R EM_Omega_Max EM_Torque_Max eff_transmission contPowerRating;

    speed = 130/3.6;    % Target vehicle speed (m/s)
    grade = 0.12;       % Road slope (12%)

    airDrag = 0.5 * Cd * fArea * airDen * (speed^2);
    rollingRes = fr * mVehicle * g * cos(atan(grade));
    gradient = mVehicle * g * sin(atan(grade));
    roadLoad = airDrag + rollingRes + gradient;
    powerReq = roadLoad * speed;

    motorRPM = speed*gearRatio*60 / (R*2*pi);
    motorOmega = motorRPM *2*pi / 60;
    motorTorque = interp1(EM_Omega_Max, EM_Torque_Max, motorRPM);
    motorPower = motorTorque * motorRPM / 9548.8;
    availablePower = motorPower * eff_transmission;

    max(EM_Omega_Max)
    if availablePower < powerReq
        suitability = 0;
    elseif powerReq > contPowerRating
        suitability = 0;
    else
        suitability = 1;
    end
end
