module Keithley2470

using Match

using ArgParse

function parse_commandline()
	s = ArgParseSettings()

	@add_arg_table! s begin
		"sleep_interupt_time"
			help = "The time interval in milliseconds that the program " *
					"sleeps for between taking samples. It might be more " *
					"performant and accurate to set this value high but " *
					"it also means that you can't cancel a measurement " *
					"in less than `sleep_interupt_time` milliseconds"
	end
	parsed_args = parse_args(s) # the result is a Dict{String,Any}
	# println("Parsed args:")
	# for (key,val) in parsed_args
	# 	println("  $key  =>  $(repr(val))")
	# end
	return parsed_args
end

import CImGui as ig, ModernGL, GLFW
import CImGui.CSyntax: @c, @cstatic
import ImPlot

global plotlock = ReentrantLock()

using NativeFileDialog, DelimitedFiles

include("BetterSleep.jl")
using .BetterSleep
import .BetterSleep: now
using Dates, TimesDates

function savetofile(times, currs, volts, timestamp_mode, filepath)
	open(filepath, "w") do io
		isempty(times) && return
		time = copy(times)
		if timestamp_mode === :datetime
			timedatenow, nanonow = TimeDate(Dates.now()), BetterSleep.now()
			synthetic_first_time = timedatenow - Nanosecond((nanonow - time[1]).ns)
			time = [synthetic_first_time + Nanosecond((tt - time[1]).ns) for tt in time]
			timeunit = "[DateTime]"
		elseif timestamp_mode === :seconds
			time = (time .- [time[1]]) .|> x->x.ns/1e9
			timeunit = "[Seconds]"
		else
			time = time .|> x->x.ns
			timeunit = "[Nanoseconds]"
		end
		writedlm(io, ["TimeStamp "*timeunit "Voltage [V]" "Current [A]"], ',')
		writedlm(io, [time volts currs], ',')
	end
end

global sleep_interupt_interval::Nano = millis(100)

using Instruments

global uwSource::GenericInstrument = GenericInstrument()
global keithley_initialized::Bool = false

# TODO add error handling by checking the keithley's error registers
function initialize_keithley()
	global uwSource
	global keithley_initialized

	rm = ResourceManager()
	instruments = find_resources(rm) # returns a list of VISA strings for all found instruments
	if length(instruments) == 0
		@error "No Instruments found, please try again with the initialize button"
		return nothing
	end
	@info "Found instruments:" instruments
	length(instruments) > 1 && @warn "More than one instrument found, connecting to first one"
	Instruments.connect!(rm, uwSource, instruments[1])
	id = query(uwSource, "*IDN?")
	@info "Instrument ID: $id"

	write(uwSource, "*RST")
	write(uwSource, "SOUR:FUNC VOLT")
	write(uwSource, "SENS:FUNC 'CURR'")
	write(uwSource, "FORM:DATA ASC")

	keithley_initialized = true
	return nothing
end

global rt_volts::Vector{Float64} = Float64[]
global rt_currs::Vector{Float64} = Float64[]
global rt_times::Vector{Nano} = Nano[]

global monitor_sample_period::Nano = millis(10)

global monitor_cancel::Ref{Bool} = Ref(false)
global monitor_is_monitoring::Ref{Bool} = Ref(false)
function keithley_monitor(volts_set::Float64, maxcurrent::Float64)
	global keithley_initialized
	if !keithley_initialized
		initialize_keithley()
		keithley_initialized || return nothing
	end

	global uwSource
	global monitor_cancel
	global monitor_sample_period
	global rt_volts
	global rt_currs
	global rt_times

	@info "Starting Monitor with voltage set to $volts_set"
	write(uwSource, "OUTP ON")
	write(uwSource, "SOUR:VOLT:ILIMIT $(maxcurrent)")

	write(uwSource, "SOUR:VOLT $(volts_set)")
	monitor_is_monitoring[] = true
	while !monitor_cancel[]
		interuptsleep(monitor_sample_period, monitor_cancel, sleep_interupt_interval)
		measIstr = query(uwSource, "MEAS:CURR?")
		measVstr = query(uwSource, "MEAS:VOLT?")
		try
			measI = parse(Float64, measIstr)
			measV = parse(Float64, measVstr)
			@lock plotlock begin
				push!(rt_times, now())
				push!(rt_currs, measI)
				push!(rt_volts, measV)
			end
		catch e
			@error e
			continue
		end
	end
	monitor_is_monitoring[] = false

	write(uwSource, "OUTP OFF")
end


global iv_volts::Vector{Float64} = Float64[]
global iv_currs::Vector{Float64} = Float64[]
global iv_times::Vector{Nano} = Nano[]


global iv_sweep_cancel::Ref{Bool} = Ref(false)
global iv_is_sweeping::Ref{Bool} = Ref(false)
function keithley_sweep(
		minvolts::Float64, maxvolts::Float64,
		stepvolts::Float64, initvolts::Float64,
		dir, maxcurrent, sweeptime::Nano)
	
	global keithley_initialized
	if !keithley_initialized
		initialize_keithley()
		keithley_initialized || return nothing
	end

	global uwSource
	global iv_sweep_cancel
	global iv_volts
	global iv_currs
	global iv_times

	write(uwSource, "OUTP ON")
	write(uwSource, "SOUR:VOLT:ILIMIT $(maxcurrent)")

	empty!(iv_volts)
	empty!(iv_currs)
	empty!(iv_times)

	if minvolts > maxvolts
		minvolts, maxvolts = maxvolts, minvolts
	end

	firstvolts = minvolts:stepvolts:maxvolts
	lastvolts = maxvolts:-stepvolts:minvolts
	volts = [firstvolts; lastvolts; minvolts]
	pivot = sortperm(abs.(volts .- initvolts))[1]
	if dir == 1
		volts = volts[[pivot:end; 1:pivot]]
	else
		volts = volts[[pivot:-1:1; end:-1:pivot]]
	end

	steptime = sweeptime/length(volts)
	iv_is_sweeping[] = true
	for (i, V) in enumerate(volts)
		iv_sweep_cancel[] && break
		write(uwSource, "SOUR:VOLT $V")
		interuptsleep(steptime, iv_sweep_cancel, sleep_interupt_interval)
		measIstr = query(uwSource, "MEAS:CURR?")
		measVstr = query(uwSource, "MEAS:VOLT?")
		measI, measV = nothing, nothing
		try
			measI = parse(Float64, measIstr)
			measV = parse(Float64, measVstr)
		catch e
			@error e
			continue
		end
		@lock plotlock begin
			push!(iv_times, now())
			push!(iv_currs, measI)
			push!(iv_volts, measV)
		end
	end
	iv_is_sweeping[] = false

	write(uwSource, "OUTP OFF")

	return nothing
end

function (@main)(ARGS)

	# Can be :datetime, :seconds, or :nanoseconds
	timestamp_mode = :datetime

	global sleep_interupt_interval
	## Parse ARGS
	parsed = parse_commandline()
	if parsed["sleep_interupt_time"] !== nothing
		sleepii = parse(Int, parsed["sleep_interupt_time"])
		sleep_interupt_interval = millis(sleepii)
	end


	## Initialize Keithley 2470
	initialize_keithley()


	## Initialize CImGui
	ig.set_backend(:GlfwOpenGL3)

	ctx = ig.CreateContext()
	io = ig.GetIO()
	io.ConfigDpiScaleFonts = true
	io.ConfigDpiScaleViewports = true
	io.ConfigFlags = unsafe_load(io.ConfigFlags) | ig.lib.ImGuiConfigFlags_DockingEnable
	io.ConfigFlags = unsafe_load(io.ConfigFlags) | ig.lib.ImGuiConfigFlags_ViewportsEnable
	style = ig.GetStyle()
	p_ctx = ImPlot.CreateContext()

	## Initialize Plot Axis Flags
	xflags = ImPlot.ImPlotAxisFlags_None | ImPlot.ImPlotAxisFlags_AutoFit
	yflags = ImPlot.ImPlotAxisFlags_None | ImPlot.ImPlotAxisFlags_AutoFit

	## Start Render Loop
	exit_application_bool = true
	first_frame = true
	ig.render(ctx; window_size=(100,100), window_title="Keithley 2470", on_exit=() -> ImPlot.DestroyContext(p_ctx)) do
		exit_application_bool || exit()

		WINSCALE = ig.GetWindowDpiScale()

		if first_frame
			win::GLFW.Window = ig._current_window(Val{:GlfwOpenGL3}())
			GLFW.HideWindow(win)
		end
		first_frame = false

		@c ig.Begin("Plot Window", &exit_application_bool,
			ig.ImGuiWindowFlags_MenuBar |
			ig.ImGuiWindowFlags_NoCollapse )

		if ig.BeginMenuBar()
			if ig.BeginMenu("Timestamp Export Mode")
				selected::Int32 = @match timestamp_mode begin
					:datetime => 1
					:seconds => 2
					:nanoseconds => 3
					_ => -1
				end

				@c ig.RadioButton("DateTime Timestamps", &selected, 1)
				@c ig.RadioButton("Seconds since start of capture", &selected, 2)
				@c ig.RadioButton("Nanoseconds since start of capture", &selected, 3)

				timestamp_mode = [:datetime, :seconds, :nanoseconds][selected]
				ig.EndMenu()
			end
			ig.EndMenuBar()
		end

		if ig.BeginTabBar("IV and RealTime", ig.ImGuiTabBarFlags_NoCloseWithMiddleMouseButton)
			if ig.BeginTabItem("I-V Sweep")

				ig.BeginGroup()
				global iv_is_sweeping
				if ig.Button("Clear Data##iv", (250WINSCALE, 30WINSCALE)) && !iv_is_sweeping[]
					ig.OpenPopup("clear_data_popup##iv")
				end
				if iv_is_sweeping[]
					if ig.BeginItemTooltip()
						ig.TextColored((255,0,0,255), "You Cannot clear data during a sweep")
						ig.EndTooltip()
					end
				end
				global iv_times
				global iv_currs
				global iv_volts
				if ig.BeginPopup("clear_data_popup##iv")
					ig.SeparatorText("Are you sure you want to erase the data?")
					ig.SeparatorText("")
					if ig.Button("I'm sure I want to permanently erase data.")
						empty!(iv_times)
						empty!(iv_currs)
						empty!(iv_volts)
						ig.CloseCurrentPopup()
					end
					ig.EndPopup()
				end

				ig.PushStyleVar(ig.lib.ImGuiStyleVar_CellPadding, (3,3))
				if ig.BeginTable("iv_maxmin_table", 2, 0, (250WINSCALE,50WINSCALE))
					ig.TableSetupColumn("Maximum [A]")
					ig.TableSetupColumn("Minimum [A]")
					ig.TableHeadersRow()
					ig.TableNextRow()
					ig.TableSetColumnIndex(0)
					ig.Text("$(isempty(iv_currs) ? "NAN" : round(maximum(iv_currs), sigdigits=5))")
					ig.TableSetColumnIndex(1)
					ig.Text("$(isempty(iv_currs) ? "NAN" : round(minimum(iv_currs), sigdigits=5))")
					ig.EndTable()
				end
				ig.PopStyleVar()

				global iv_is_sweeping
				if ig.Button("Save Data##iv", (250WINSCALE, 30WINSCALE)) && !iv_is_sweeping[]
					filepath = save_file(;filterlist="csv")
					!isempty(filepath) && savetofile(iv_times, iv_currs, iv_volts, timestamp_mode, filepath)
				end
				if iv_is_sweeping[]
					if ig.BeginItemTooltip()
						ig.TextColored((255,0,0,255), "You cannot save data during a sweep")
						ig.EndTooltip()
					end
				end

				ig.PushItemWidth(90WINSCALE)
				@cstatic iv_min_volts=Cdouble(-1) iv_max_volts=Cdouble(1) iv_step_voltage=Cdouble(0.1) iv_sweep_time=Cdouble(0) iv_init_voltage=Cdouble(0) maxcurrent=Cdouble(0.1) iv_sweep_dir=Int32(1) begin
				
				@c ig.InputDouble("Minimum Voltage [V]", &iv_min_volts)
				@c ig.InputDouble("Maximum Voltage [V]", &iv_max_volts)
				@c ig.InputDouble("Step Voltage [V]", &iv_step_voltage)
				if iv_step_voltage < 0 iv_step_voltage = 0 end
				@c ig.InputDouble("Sweep time [s]", &iv_sweep_time)
				if iv_sweep_time < 0 iv_sweep_time = 0 end
				
				@c ig.InputDouble("Initial Voltage [V]", &iv_init_voltage)
				@c ig.InputDouble("Max Current [A]", &maxcurrent)
				ig.PopItemWidth()
				
				ig.SetNextItemWidth(170WINSCALE)
				@c ig.Combo("Start Sweep##combo", &iv_sweep_dir, ["Towards Positive", "Towards Negative"])

				global monitor_is_monitoring
				global iv_is_sweeping
				global iv_cancel_sweep
				if !iv_is_sweeping[] && ig.Button("Start Sweep", (250WINSCALE, 30WINSCALE)) && !monitor_is_monitoring[]
					ig.OpenPopup("start_sweep_popup")
				end
				if monitor_is_monitoring[]
					if ig.BeginItemTooltip()
						ig.TextColored((255,0,0,255), "You Cannot start a sweep while monitoring")
						ig.EndTooltip()
					end
				end
				if ig.BeginPopup("start_sweep_popup")
					ig.SeparatorText("Are you sure you want to start a sweep?")
					ig.SeparatorText("Starting a sweep will erase the previous sweep from memory.")
					if ig.Button("I'm sure I want to permanently erase data and start a new sweep.")
						iv_cancel_sweep = false
						errormonitor(
							Threads.@spawn keithley_sweep(
								iv_min_volts, iv_max_volts,
								iv_step_voltage, iv_init_voltage,
								iv_sweep_dir, maxcurrent, seconds(iv_sweep_time))
						)
						ig.CloseCurrentPopup()
					end
					ig.EndPopup()
				end
				if iv_is_sweeping[] && ig.Button("Cancel Sweep##iv_sweep")
					iv_cancel_sweep = true
				end
				end #= @cstatic iv_min_volts=Cdouble(-1) iv_max_volts=Cdouble(1) iv_sweep_time=Cdouble(0) iv_init_voltage=Cdouble(0) maxcurrent=Cdouble(1) iv_sweep_dir=Int32(1) begin =#


				ig.EndGroup()
				
				ig.SameLine()

				ig.BeginGroup()
				@c ig.CheckboxFlags("Fit X-Axis", &xflags, ImPlot.ImPlotAxisFlags_AutoFit)
				ig.SameLine()
				@c ig.CheckboxFlags("Fit Y-Axis", &yflags, ImPlot.ImPlotAxisFlags_AutoFit)
				if (xflags | yflags) & ImPlot.ImPlotAxisFlags_AutoFit != 0
					if (xflags & yflags) & ImPlot.ImPlotAxisFlags_AutoFit != 0
						xflags = xflags & ~ImPlot.ImPlotAxisFlags_RangeFit
						yflags = yflags & ~ImPlot.ImPlotAxisFlags_RangeFit
					else
						ig.SameLine()
						if xflags & ImPlot.ImPlotAxisFlags_AutoFit != 0
							@c ig.CheckboxFlags("Range Fit", &xflags, ImPlot.ImPlotAxisFlags_RangeFit)
						elseif yflags & ImPlot.ImPlotAxisFlags_AutoFit != 0
							@c ig.CheckboxFlags("Range Fit", &yflags, ImPlot.ImPlotAxisFlags_RangeFit)
						end
					end
				end

				if ImPlot.BeginPlot("I-V Sweep", "Voltage [V]", "Current [A]", ig.ImVec2(-1,-1))
					ImPlot.SetupAxes("Voltage [V]", "Current [A]", xflags, yflags)
					if !isempty(iv_currs)
						@lock plotlock begin
							ImPlot.PlotLine("data", iv_currs, iv_volts)
						end
					end
					ImPlot.EndPlot()
				end
				
				ig.EndGroup()

				ig.EndTabItem()
			end
			if ig.BeginTabItem("Realtime Monitor")

				ig.BeginGroup()
				global monitor_is_monitoring
				if ig.Button("Clear Data##rt", (250WINSCALE, 30WINSCALE))
					ig.OpenPopup("clear_data_popup##rt")
				end
				global rt_times
				global rt_currs
				global rt_volts
				if ig.BeginPopup("clear_data_popup##rt")
					ig.SeparatorText("Are you sure you want to erase the data?")
					ig.SeparatorText("")
					if ig.Button("I'm sure I want to permanently erase data.")
						empty!(rt_times)
						empty!(rt_currs)
						empty!(rt_volts)
						ig.CloseCurrentPopup()
					end
					ig.EndPopup()
				end

				ig.PushStyleVar(ig.lib.ImGuiStyleVar_CellPadding, (3,3))
				if ig.BeginTable("iv_maxmin_table", 3, 0, (250WINSCALE,50WINSCALE))
					ig.TableSetupColumn("Maximum [A]")
					ig.TableSetupColumn("Minimum [A]")
					ig.TableSetupColumn("Average [A]")
					ig.TableHeadersRow()
					ig.TableNextRow()
					ig.TableSetColumnIndex(0)
					ig.Text("$(isempty(rt_currs) ? "NAN" : round(maximum(rt_currs), sigdigits=5))")
					ig.TableSetColumnIndex(1)
					ig.Text("$(isempty(rt_currs) ? "NAN" : round(minimum(rt_currs), sigdigits=5))")
					ig.TableSetColumnIndex(2)
					ig.Text("$(isempty(rt_currs) ? "NAN" : round(sum(rt_currs)/length(rt_currs), sigdigits=5))")
					ig.EndTable()
				end
				ig.PopStyleVar()

				global monitor_is_monitoring
				if ig.Button("Save Data##rt", (250WINSCALE, 30WINSCALE))
					filepath = save_file(;filterlist="csv")
					!isempty(filepath) && savetofile(rt_times, rt_currs, rt_volts, timestamp_mode, filepath)
				end

				ig.PushItemWidth(90WINSCALE)
				@cstatic set_volts=Cdouble(1) samplerate=Cdouble(0.001) maxcurrent=Cdouble(0.1) begin
				
				@c ig.InputDouble("Set Voltage [V]", &set_volts)
				@c ig.InputDouble("Sample rate [s]", &samplerate)
				if samplerate < 0 samplerate = 0 end
				global monitor_sample_period
				monitor_sample_period = seconds(samplerate)

				@c ig.InputDouble("Max Current [A]", &maxcurrent)
				ig.PopItemWidth()

				global monitor_is_monitoring
				global monitor_cancel
				global iv_is_sweeping
				if !monitor_is_monitoring[]
					if !isempty(rt_times)
						if ig.Button("Resume", (250WINSCALE, 40WINSCALE)) && !iv_is_sweeping[]
							monitor_cancel[] = false
							errormonitor(Threads.@spawn keithley_monitor(set_volts, maxcurrent))
						end
					elseif ig.Button("Start", (250WINSCALE, 40WINSCALE)) && !iv_is_sweeping[]
						monitor_cancel[] = false
						errormonitor(Threads.@spawn keithley_monitor(set_volts, maxcurrent))
					end
				else
					if ig.Button("Stop", (250WINSCALE, 40WINSCALE))
						monitor_cancel[] = true
					end
				end

				end #= @cstatic set_volts=Cdouble(1) samplerate=Cdouble(0.001) maxcurrent=Cdouble(0.1) begin =#


				ig.EndGroup()

				ig.SameLine()

				ig.BeginGroup()
				@c ig.CheckboxFlags("Fit X-Axis", &xflags, ImPlot.ImPlotAxisFlags_AutoFit)
				ig.SameLine()
				@c ig.CheckboxFlags("Fit Y-Axis", &yflags, ImPlot.ImPlotAxisFlags_AutoFit)
				if (xflags | yflags) & ImPlot.ImPlotAxisFlags_AutoFit != 0
					if (xflags & yflags) & ImPlot.ImPlotAxisFlags_AutoFit != 0
						xflags = xflags & ~ImPlot.ImPlotAxisFlags_RangeFit
						yflags = yflags & ~ImPlot.ImPlotAxisFlags_RangeFit
					else
						ig.SameLine()
						if xflags & ImPlot.ImPlotAxisFlags_AutoFit != 0
							@c ig.CheckboxFlags("Range Fit", &xflags, ImPlot.ImPlotAxisFlags_RangeFit)
						elseif yflags & ImPlot.ImPlotAxisFlags_AutoFit != 0
							@c ig.CheckboxFlags("Range Fit", &yflags, ImPlot.ImPlotAxisFlags_RangeFit)
						end
					end
				end
				if ImPlot.BeginPlot("Realtime Monitor", "Time [ns]", "Current [A]", ig.ImVec2(-1, -1))
					ImPlot.SetupAxes("Time [s]", "Current [A]", xflags, yflags)
					if !isempty(rt_currs)
						@lock plotlock begin
							F = first(rt_times)
							xs = rt_times .|> x->(x.ns-F.ns)/1e9
							ImPlot.PlotLine("data", xs, rt_currs)
						end
					end
					ImPlot.EndPlot()
				end

				ig.EndGroup()

				ig.EndTabItem()
			end

			ig.EndTabBar()
		end


		ig.End()
	end

	write(uwSource, "OUTP OFF")

	return 0
end


end # module Keithley2470
