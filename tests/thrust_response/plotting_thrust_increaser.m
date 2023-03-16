data = readtable("increasing_thrust_response.csv");

% Correct the data
data.Var1 = (data.Var1-5)*100;
data.Var2 = (0.82-data.Var2)*9.82;

% Fit data to a first order polynomial
xdata = data(550:1280,:);
[p,S] = polyfit(xdata.Var1,xdata.Var2,1)

% create vectors for plotting
X = linspace(min(xdata.Var1),max(xdata.Var1),100);
Y = polyval(p,X);

% plot
hold on
plot(data.Var1,data.Var2,'o')
plot(X,Y,'LineWidth',4)
xlim([60 2000])
xlabel("Input integer [-]")
ylabel("Thrust [N]")
text(200,7,"y=(4.436e-3)x-1.6")

4.436

grid on