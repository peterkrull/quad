data = readtable("../../tests/increasing_thrust_response.csv");

data.Var1 = (data.Var1-5)*100
data.Var2 = (0.82-data.Var2)*9.82

% X = 4:0.1:27;
% Y = 0.0465*X-0.415

xdata = data(550:1280,:)

[p,S] = polyfit(xdata.Var1,xdata.Var2,1)

X = linspace(min(xdata.Var1),max(xdata.Var1),100);
Y = polyval(p,X);
 
plot(data.Var1,data.Var2,'o',X,Y,'LineWidth',2)

xlim([0 2000])

xlabel("Input integer [-]")
ylabel("Thrust [N]")

grid on