using ControlSystems,Plots,CSV,DataFrames


data0 = CSV.read("tests/thrust_response/1500_thrust_response.csv",DataFrame,header=["time","thrust"])
data1 = CSV.read("tests/thrust_response/1200_thrust_response.csv",DataFrame,header=["time","thrust"])
data2 = CSV.read("tests/thrust_response/0500_thrust_response.csv",DataFrame,header=["time","thrust"])

# measured at a battery voltage of : 15.82 V

X = 0:0.01:2;
Y = 1 .-exp.(-X/0.05); # tau found to be 50 ms

p = plot(X,Y)
scatter!(p,data0.time.-2.9,(0.800.-data0.thrust)/0.584,label = "1500")
scatter!(p,data1.time.-3.1,(0.793.-data1.thrust)/0.41,label = "1200")
scatter!(p,data2.time.-3.7,(0.792.-data2.thrust)/0.093,label = "0500")

xlabel!("Time [s]")
ylabel!("Normalized thrust [-]")

title!("tau = 0.05 s")

xlims!(p,(-0.5, 1))
ylims!(p,(-0.1, 1.2))
plot!(size=(800,500))

savefig(p,"images/thrust_step_plot.svg")