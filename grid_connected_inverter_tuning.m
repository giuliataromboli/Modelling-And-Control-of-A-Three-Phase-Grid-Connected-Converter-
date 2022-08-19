clear all
clc

%% DATA
omega=2*pi*50;
V_dc=690;
A_inv=50e3;
I_max=A_inv/V_dc;
I=0;

switching_frequency=2500;
sampling_time=1/(2*switching_frequency);
T=1.5*sampling_time;

L=1e-3;
R=0.314;
C=6e-3;
C_dc=10e-3;

fault=0;

if fault == 1
    V_a=0;
    V_b=sqrt(3)*400;
    V_c=sqrt(3)*400;
    phi_a=0;
    phi_b=-5/6*pi;
    phi_c=5/6*pi;
else
    V_a=400;
    V_b=400;
    V_c=400;
    phi_a=0;
    phi_b=-2/3*pi;
    phi_c=-4/3*pi;
end

%PLL
settling_time=1;
damping=1;

Kp_PLL=9.2/settling_time;
Ti_PLL=settling_time*damping^2/2.3;
Ki_PLL=Kp_PLL/Ti_PLL;

%% TRANSFER FUNCTION
%PWM
PWM=tf([-T 2],[T 2]);

%RL filter
RL_filter=tf([1],[L R]);

%DC-link
DC_link=tf([1],[C_dc/2 0]);

%Bode Option
p=bodeoptions;
p.PhaseVisible='off';
p.XLimMode='manual';
p.XLim={[10^(-4),10*10^4]};
p.YLimMode='manual';
p.YLim={[-100,100]};
p.FreqUnits='Hz';

%% TUNING CURRENT
rise_time_current=3e-3;
alpha_current=log(9)/rise_time_current;
T_cur=1/alpha_current;

%PI controller
Kp_current=alpha_current*L;
Ki_current=alpha_current*R;
Ti_current=Kp_current/Ki_current;
PI_current=tf([Kp_current Ki_current],[1 0]);

%Open loop analysis
G_co=PI_current*PWM*RL_filter;

figure(1)
nyquist(G_co)
grid on
xlim([-2;2])
ylim([-2;2])

%Closed loop analysis
G_cc=G_co/(1+G_co);
sensitivity_current=1/(1+G_co);
noise_sensitivity_current=PI_current/(1+G_co);
load_sensitivity_current=(PWM*RL_filter)/(1+G_co);

figure(2)
bode(sensitivity_current,p);
hold on
bode(G_cc,p);
bode(noise_sensitivity_current,p);
bode(load_sensitivity_current,p);
hold off
grid on
legend('Sensitivity','Complementary sensitivity','Noise sensitivity','Load sensitivity','location','southeast')

figure(3)
step(G_cc)
xlim([0;0.05])
ylim([0;1.5])
grid on


%% TUNING DC-VOLTAGE
%PI controller
a=3;
Ti_DC_voltage=(a^2)*T_cur;
Kp_DC_voltage=1;
Ki_DC_voltage=Kp_DC_voltage/Ti_DC_voltage;
PI_DC_voltage=tf([Kp_DC_voltage Ki_DC_voltage],[1 0]);

%Open loop analysis
G_vo=PI_DC_voltage*G_cc*DC_link;

figure(4)
rlocus(G_vo)
grid on
xlim([-4000;4000])
ylim([-4000;4000])

figure(5)
nyquist(G_vo)
grid on
xlim([-2;2])
ylim([-2;2])

%Closed loop analysis
G_vc=G_vo/(1+G_vo);
sensitivity_DC_voltage=1/(1+G_vo);
noise_sensitivity_DC_voltage=PI_DC_voltage/(1+G_vo);
load_sensitivity_DC_voltage=(G_cc*DC_link)/(1+G_vo);

figure(6)
bode(sensitivity_DC_voltage,p);
hold on
bode(G_vc,p);
bode(noise_sensitivity_DC_voltage,p);
bode(load_sensitivity_DC_voltage,p);
hold off
grid on
legend('Sensitivity','Complementary sensitivity','Noise sensitivity','Load sensitivity','location','southeast')
grid on

figure(7)
step(G_vc)
xlim([0;0.2])
ylim([-0.5;1.5])
grid on

