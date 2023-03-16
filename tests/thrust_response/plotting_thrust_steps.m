data0 = readtable("1500_thrust_response.csv") % 100 -> 1500
data1 = readtable("1200_thrust_response.csv") % 100 -> 1200
data2 = readtable("0500_thrust_response.csv") % 100 -> 500

% measured at a battery voltage of : 15.82 V

X = 0:0.01:2;
Y = 1-exp(-X/0.05); % tau found to be 50 ms

hold on
plot(data0.Var1-2.9,(0.800-data0.Var2)/0.584,'x',X,Y)
plot(data1.Var1-3.1,(0.793-data1.Var2)/0.41,'*')
plot(data2.Var1-3.7,(0.792-data2.Var2)/0.093,'o')

xlabel('Time [s]')
ylabel('Normalized thrust [-]')

text(-0.35,1,'tau = 0.05','FontSize',12)

xlim([-0.5 1])
ylim([-0.1 1.2])
grid on

exportgraphics(gcf,'thrust_step_plot.pdf','ContentType','vector')