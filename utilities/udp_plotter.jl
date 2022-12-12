if length( ARGS ) < 1
    println("No file name provided; logging disabled.")
end

@time begin
using Sockets, Plots
using DataFrames, CSV
include("data_structures.jl")
end

using PackageCompiler

function plotter()::Cint
    @time begin
        gr(reuse=true)
    end
    
    frequency_hz = 50
    window_time = 50
    
    buffer_len = frequency_hz*window_time
    
    df = DataFrame([
        "pitch_con"=>zeros(buffer_len),"roll_con"=>zeros(buffer_len),"yaw_con"=>zeros(buffer_len),
        "pitch_vcon"=>zeros(buffer_len),"roll_vcon"=>zeros(buffer_len),"yaw_vcon"=>zeros(buffer_len),
        "motor1"=>zeros(buffer_len),"motor2"=>zeros(buffer_len),"motor3"=>zeros(buffer_len),"motor4"=>zeros(buffer_len),
        "radio_thrust"=>zeros(buffer_len),"radio_pitch"=>zeros(buffer_len),"radio_roll"=>zeros(buffer_len),"radio_yaw"=>zeros(buffer_len),
        "accx"=>zeros(buffer_len),"accy"=>zeros(buffer_len),"accz"=>zeros(buffer_len),
        "gyrox"=>zeros(buffer_len),"gyroy"=>zeros(buffer_len),"gyroz"=>zeros(buffer_len),
        "pitch"=>zeros(buffer_len),"roll"=>zeros(buffer_len),"yaw"=>zeros(buffer_len),
        "swE"=>zeros(buffer_len),"swB"=>zeros(buffer_len),"swC"=>zeros(buffer_len),"swF"=>zeros(buffer_len),
    ])

    x = (1/frequency_hz):(1/frequency_hz):window_time;

    # @async begin
    @time begin
    udpsock = UDPSocket()
    bind(udpsock,ip"0.0.0.0",50000)
    println(size(recv(udpsock)))
    end

    buffer_counter = 1

    keep_alive = true
    try
    while keep_alive      
            
        data :: drone_data = reinterpret(drone_data,recv(udpsock))[1]

        dfx = DataFrame([
            "pitch_con"=>data.pitch_control[1],"roll_con"=>data.roll_control[1],"yaw_con"=>data.yaw_control[1],
            "pitch_vcon"=>data.pitch_control[2],"roll_vcon"=>data.roll_control[2],"yaw_vcon"=>data.yaw_control[2],
            "motor1"=>data.motor_speeds[1],"motor2"=>data.motor_speeds[2],"motor3"=>data.motor_speeds[3],"motor4"=>data.motor_speeds[4],
            "radio_thrust"=>data.radio_control[1],"radio_pitch"=>data.radio_control[2],"radio_roll"=>data.radio_control[3],"radio_yaw"=>data.radio_control[4],
            "accx"=>data.acc[1],"accy"=>data.acc[2],"accz"=>data.acc[3],
            "gyrox"=>data.gyro[1],"gyroy"=>data.gyro[2],"gyroz"=>data.gyro[3],
            "pitch"=>data.pry[1],"roll"=>data.pry[2],"yaw"=>data.pry[3],
            "swE"=>data.switches[1],"swB"=>data.switches[2],"swC"=>data.switches[3],"swF"=>data.switches[4],
        ])

        
        df[buffer_counter,:] = dfx[1,:]

        buffer_counter += 1
        if buffer_counter > buffer_len
            buffer_counter = 1
        end

        if buffer_counter%5 == 0
            p = plot(size = (1920, 1080))
            
            for (col,name) in zip(eachcol(df),names(df))
                if name ∈ ["pitch","roll","yaw"]
                    plot!(p,collect(x),[ col[buffer_counter:end] ; col[1:buffer_counter-1] ],label=name,legend=:left)
                end
            end
            display(p)
        end

        if round(dfx.swF[1]) == 2 keep_alive = false end
    end
    catch e
        if e isa(InterruptException)
            println("Stopping..")
        end

    end

    CSV.write(ARGS[1],[ df[buffer_counter:end,:] ; df[1:buffer_counter-1,:] ])
    return 0
end

plotter()