%% Fixed rotor test
close all; clear

motordata = importDriveData('fixedrotordata.txt',2);
motordata.Properties.VariableNames{1} = 'Time_sec';
motordata.Properties.VariableNames{2} = 'Current_mA';

voltage_supply_V = 8.866;
duty_cycles = [0.1,0.2,0.3,0.4,0.5];

lastValidIdx = find(motordata.Time_sec <= 22,1,"last");

plottingOn = 1;
if plottingOn
    figure;
    plot(motordata.Time_sec(1:lastValidIdx),motordata.Current_mA(1:lastValidIdx))
    grid on
    xlabel('Time $t$ (sec)','Interpreter','latex')
    ylabel('Current $i_m$ (mA)','Interpreter','latex')
    title('Fixed Rotor Test: Armature Current','Interpreter','latex')
end

inRange = @(timevec,x1,x2) all([~le(timevec,x1),~ge(timevec,x2)],2);
getCurrent_A = @(currentvec,timevec,x1,x2) mean(currentvec(inRange(timevec,x1,x2)))/1000;

current_bias(1) = getCurrent_A(motordata.Current_mA,motordata.Time_sec,0,2);
current_bias(2) = getCurrent_A(motordata.Current_mA,motordata.Time_sec,5.1,7);
current_bias(3) = getCurrent_A(motordata.Current_mA,motordata.Time_sec,10.1,12);

current_meanBias_A = mean(current_bias);

dc_current_A(1) = getCurrent_A(motordata.Current_mA,motordata.Time_sec,2.1,5) - current_meanBias_A;
dc_current_A(2) = getCurrent_A(motordata.Current_mA,motordata.Time_sec,7.1,10) - current_meanBias_A;
dc_current_A(3) = getCurrent_A(motordata.Current_mA,motordata.Time_sec,12.1,15) - current_meanBias_A;
dc_current_A(4) = getCurrent_A(motordata.Current_mA,motordata.Time_sec,17.1,20) - current_meanBias_A;
dc_current_A(5) = 0; % Data is invalid for this case

resistance_ohm = duty_cycles*voltage_supply_V./dc_current_A;
% Calculate mean resistance to use downstream
resistance_mean_ohm = mean(resistance_ohm(~isinf(resistance_ohm)));

%% PWM Duty Cycle Sweep Test
% Yellow is M1, No tag is M2, Blue is M3, Red is M4
drivedata = importDriveData('motordata.txt',3);
drivedata.Properties.VariableNames{1} = 'Time_sec';
drivedata.Properties.VariableNames{2} = 'Velocity_dps';
drivedata.Properties.VariableNames{3} = 'Current_mA';

transientTestFlag = 0;
transientTestDC = 0.3;
t_PWM_On = 2;

for idx = 1:length(drivedata.Time_sec)
    if drivedata.Time_sec(idx) >= 2 && drivedata.Time_sec(idx) < 8 
        drivedata.DutyCycle(idx) = 0.15;
    elseif drivedata.Time_sec(idx) >= 10 && drivedata.Time_sec(idx) < 16
        drivedata.DutyCycle(idx) = 0.3;
    elseif drivedata.Time_sec(idx) >= 18 && drivedata.Time_sec(idx) < 24
        drivedata.DutyCycle(idx) = 0.45;
    elseif drivedata.Time_sec(idx) >= 26 && drivedata.Time_sec(idx) < 32
        drivedata.DutyCycle(idx) = 0.6;
    else
        drivedata.DutyCycle(idx) = 0;
    end
end

plottingOn = 1;
if plottingOn
    figure;
    subplot(3,1,1)
    plot(drivedata.Time_sec,drivedata.DutyCycle)
    grid on
    xlabel('Time $t$ (sec)',"Interpreter","latex")
    ylabel('PWM Duty Cycle $u(t)$',"Interpreter","latex")
    ylim([0 1])
    subplot(3,1,2)
    plot(drivedata.Time_sec,drivedata.Current_mA)
    grid on
    xlabel('Time $t$ (sec)',"Interpreter","latex")
    ylabel('Current $i_m$ (mA)',"Interpreter","latex")
    subplot(3,1,3)
    plot(drivedata.Time_sec,drivedata.Velocity_dps)
    grid on
    xlabel('Time $t$ (sec)',"Interpreter","latex")
    ylabel('Velocity $\omega_m$ (dps)',"Interpreter","latex")
    sgtitle('PWM Duty Cycle Motor Test', 'Interpreter', 'latex', 'FontSize', 16);
end

w_radpsec = drivedata.Velocity_dps*pi/180;
getVelocity_radpsec = @(timevec,x1,x2) mean(w_radpsec(inRange(timevec,x1,x2)));
% To calculate r_m, use expression for v_m at steady state
if transientTestFlag
    duty_cycles_dyn = transientTestDC;
    rangeMat = [2.1,8];
else
    duty_cycles_dyn = [0.15,0.30,0.45,0.60];
    rangeMat = [2.1,8;10.2,16;18.2,24;26.2,32];
end

for idx = 1:size(rangeMat,1)
    i_m(idx) = getCurrent_A(drivedata.Current_mA,drivedata.Time_sec,...
        rangeMat(idx,1),rangeMat(idx,2))-current_meanBias_A;
    v_r = resistance_mean_ohm*i_m(idx);
    v_m(idx) = voltage_supply_V*duty_cycles_dyn(idx) - v_r;
    w_m(idx) = getVelocity_radpsec(drivedata.Time_sec,rangeMat(idx,1),rangeMat(idx,2));
end

if ~transientTestFlag
    lin_model_elec = polyfit(w_m,v_m,1);
    r_m = lin_model_elec(1); v_m_0 = lin_model_elec(2);
    v_m_fit = v_m_0 + r_m*w_m;
    figure
    subplot(1,2,1)
    plot(w_m,v_m,'-ob',"DisplayName","Data based")
    grid on
    xlabel('Rotor Speed $\omega_m$ (rad/sec)',"Interpreter","latex")
    ylabel('Motor Voltage $v_m$ (V)',"Interpreter","latex")
    hold on; plot(w_m,v_m_fit,'-r',"DisplayName","Linear fit")
    legend('Interpreter','latex')
    title(strcat('Motor constant $r_m$ = ',num2str(r_m),' $V*s/rad$'),"Interpreter","latex")
    hold off;
    rmValid = 1;
    % Find Bm at steady state
    lin_model_mech = polyfit(w_m,r_m*i_m,1);
    B_m = lin_model_mech(1); T_m_0 = lin_model_mech(2);
    T_m_fit = T_m_0 + B_m*w_m;
    subplot(1,2,2)
    plot(w_m,r_m*i_m,'-ob',"DisplayName","Data based")
    grid on
    xlabel('Rotor Speed $\omega_m$ (rad/sec)',"Interpreter","latex")
    ylabel('Motor Torque $T_m$ (Nm)',"Interpreter","latex")
    hold on; plot(w_m,T_m_fit,'-r',"DisplayName","Linear fit")
    legend('Interpreter','latex')
    title(strcat('Motor damping constant $B_m$ = ',num2str(B_m),' $Nm*s/rad$'),"Interpreter","latex")
    hold off;
    BmValid = 1;
else
    load("Data\MotorParameters.mat");
end

% Find J of motor from transient response of omega curve
if transientTestFlag && BmValid
    w_tau = 0.632*w_m;
    jdx = find(w_radpsec>=w_tau(1),1,"first");
    tau = interp1([w_radpsec(jdx-1),w_radpsec(jdx)],...
        [drivedata.Time_sec(jdx-1),drivedata.Time_sec(jdx)],...
        w_tau) - t_PWM_On;
    % tau = time @ w_tau;
    J = B_m*tau;
    save("Data\MotorInertia.mat","J")
end
%% Generate torque speed curves
if ~transientTestFlag
    figure
    hold on
    omega_radpsec = 0:20;
    opts = {'-b','-r','-g','-m'};
    for idx = 1:length(duty_cycles_dyn)
        v_in = duty_cycles_dyn(idx)*voltage_supply_V;
        T_o(idx,:) = (r_m/resistance_mean_ohm)*v_in...
            - (r_m^2/resistance_mean_ohm+B_m)*omega_radpsec;
        plot(omega_radpsec,T_o(idx,:),opts{idx},...
            "DisplayName",strcat("$v_{in}$ = ",num2str(v_in)," V"))
    end
    hold off
    grid on
    xlabel('Rotor Speed $\omega_m$ (rad/sec)',"Interpreter","latex")
    ylabel('Motor Torque $T_m$ (Nm)',"Interpreter","latex")
    legend('Interpreter','latex')
    title('Torque Speed Curves at Testing Voltages',...
        "Interpreter","latex")
    xlim([0 20])
    ylim([0 0.1])
end

%% Export parameters to .mat file
if ~transientTestFlag
    save("Data\MotorParameters.mat","B_m","r_m","T_m_0","resistance_mean_ohm","rmValid","BmValid")
    save("Data\DriveData.mat","drivedata","current_meanBias_A")
end
