using ControlSystems,Plots,CSV,DataFrames,CurveFit

data = CSV.read("tests/thrust_response/increasing_thrust_response.csv",DataFrame,header=["time","thrust"])

# Correct the data
data.time = (data.time.-5)*100;
data.thrust = (0.82.-data.thrust)*9.82;

# Fit data to a first order polynomial
xdata = data[550:1280,:]
b,a = linear_fit(xdata.time,xdata.thrust)

# create vectors for plotting
X = minimum(xdata.time):maximum(xdata.time)
Y = a*X .+ b

# plot
p = plot(data.time,data.thrust, label = "Measured thrust")
plot!(p,X,Y,linewidth = 5, label = "Linear fit")
xlims!((60, 2000))
xlabel!("Input integer [-]")
ylabel!("Thrust [N]")
title!("y=(4.437e-3)x-1.6")

plot!(size=(800,500))

savefig(p,"images/thrust_linearization.svg")
