NUMSTUDY = 500
include("constants.jl")
include("rendering.jl")
using PyCall
unshift!(PyVector(pyimport("sys")["path"]), os.path[:join](SUMO_HOME, "tools"))
@pyimport sys
@pyimport os
@pyimport traci
@pyimport traci.constants as tc
@pyimport sumolib
screen_width = 1000
screen_height = 1000
side_screen_width, side_screen_height = 800, 1000
world_width = XBOUNDRIGHT-XBOUNDLEFT
world_widthx = XBOUNDRIGHT-XBOUNDLEFT
scalex = screen_width/world_widthx
world_widthy = YBOUNDTOP-YBOUNDBOT
scaley = screen_width/world_widthy


type Status
 start::String
 dest::String
 step::Array{Int,1}
 edge::Array{String,1}
 pos::Array{Tuple{Float64,Float64},1}
 speed::Array{Float64,1}
 arrived::Bool
end
type BackStatus
 step::Int
 edge::Array{String, 1}
 speed::Array{Float64, 1}
 starttime::Array{Int, 1}
 endtime::Array{Int, 1}
 startspeed::Array{Float64, 1}
 endspeed::Array{Float64, 1}
 meanspeed::Array{Float64, 1}
 stdspeed::Array{Float64, 1}
 arrived::Bool
end
type EdgeStatus
 step::Array{Int, 1}
 traveltime::Array{Float64, 1}
 travelspeed::Array{Float64, 1}
 spdlimit::Float64
end
type CarHandler
    xs::Array{Float64, 1}
    ys::Array{Float64, 1}
    cidxs::Array{Int, 1}
    isshowns::Array{Bool, 1}
end
function update_geom(handler::CarHandler, id::Int, x::Float64, y::Float64, cidx::Int)
    if id > length(handler.xs)
        push!(handler.xs, x*scalex-5)
        push!(handler.ys, y*scaley+45)
        push!(handler.cidxs, cidx)
        push!(handler.isshowns, true)
    else
        handler.xs[id] = x*scalex-5
        handler.ys[id] = y*scaley+45
        handler.cidxs[id] = cidx
    end
end
vehicleStatus = Dict()
vehicleVisual = Dict()
backgroundStatus = Dict()
netStatus = Dict()

ANNARBOR = sumolib.net[:readNet]("/media/mSATA/UM/Simulation/f_AnnArbor1.6.net.xml")
EDGES = ANNARBOR[:getEdges]()
NET = Dict{String,Int}()
for e in EDGES
    NET[e[:getID]()]  = -1
end
open("/media/mSATA/UM/Simulation/finalclusters.txt") do f
    this_zone = -1
    for line in eachline(f)
        res = split(line)
        if res[1] == "Zone"
            this_zone = parse(Int, res[2])
        elseif res[1] == "Links"
            for lidx in res[2:end]
                ln = data_links[:link][parse(Int, lidx)]
                if string(ln, "-0") in keys(NET)
                    NET[string(ln, "-0")] = this_zone
                end
                if string(ln, "-1") in keys(NET)
                    NET[string(ln, "-1")] = this_zone
                end
                if string(ln, "-1-Pocket") in keys(NET)
                    NET[string(ln, "-1-Pocket")] = this_zone
                end
                if string(ln, "-0-Pocket") in keys(NET)
                    NET[string(ln, "-0-Pocket")] = this_zone
                end
            end
        end
    end
end
function _checkBackInitialPositions(vehicleID, start, dest, edge, pos, time, speed)
 if !(vehicleID in keys(backgroundStatus))
   backgroundStatus[vehicleID] = Status(start, dest, [Int(time/1000)], [edge], [pos], [speed], dest==edge)
 end
end
function _checkInitialPositions(vehicleID, start, dest, edge, pos, time, speed, viewer, cars, carhandler)
 if !(vehicleID in keys(vehicleStatus))
  # a new vehicle not in the analysis list
  if length(vehicleStatus) < NUMSTUDY && start == edge
   # make sure this new vehicle is the first 200 cars and at the start of its route
   vehicleStatus[vehicleID] = Status(start, dest, [Int(time/1000)], [edge], [pos], [speed], dest==edge)
   #vehicleVisual[vehicleID] = Image("/home/boqi/Pictures/noun_Car_982516.png", 25, 25, pos[1]*scale-15, pos[2]*scale-15)
   #viewer[:add_geom](vehicleVisual[vehicleID])
   vehicleVisual[vehicleID] = length(vehicleVisual) + 1
   update_geom(carhandler, vehicleVisual[vehicleID], pos[1], pos[2], 1)
   #cars[:update_geom](vehicleVisual[vehicleID], pos[1]*scale-15, pos[2]*scale-15, 1)
  end
 end
end

function doStep(viewer, cars, carhandler, datas)
    traci.simulationStep()
    #moveNodes = []
    subscribes = traci.vehicle[:getSubscriptionResults]()
    time = traci.simulation[:getCurrentTime]()

    #count = 0
    if time%60 == 0
        datas["bar2"] = zeros(Int, 14)
        datas["bar3"] = zeros(14)
        datas["bar3count"] = ones(14)*0.000001
        for (veh, subs) in subscribes
            edge = traci.vehicle[:getRoadID](veh)
            speed = traci.vehicle[:getSpeed](veh)
            datas["bar2"][NET[edge]] += 1
            datas["bar3"][NET[edge]] += speed
            datas["bar3count"][NET[edge]] += 1
        end
    end
    departed = traci.simulation[:getSubscriptionResults]()[tc.VAR_DEPARTED_VEHICLES_IDS]
    arrived = traci.simulation[:getSubscriptionResults]()[tc.VAR_ARRIVED_VEHICLES_IDS]
    for v in departed
        traci.vehicle[:subscribe](v)
        #subs = traci.vehicle[:getSubscriptionResults](v)
        route = traci.vehicle[:getRoute](v)
        speed = traci.vehicle[:getSpeed](v)
        pos = traci.vehicle[:getPosition](v)
        start = route[1]
        dest = route[end]
        edge = traci.vehicle[:getRoadID](v)
        #push!(moveNodes, (v, route[1], route[end], subs[tc.VAR_ROAD_ID], pos, speed))
        _checkBackInitialPositions(v, start, dest, edge, pos, time, speed)
        _checkInitialPositions(v, start, dest, edge, pos, time, speed, viewer, cars, carhandler)
        datas["bar1"][NET[start]][1] += 1
        datas["bar1"][NET[dest]][2] += 1
    end
    for v in arrived
        datas["bar1"][NET[backgroundStatus[v].start]][1] -= 1
        datas["bar1"][NET[backgroundStatus[v].dest]][2] -= 1
    end
    for vehicleID in keys(vehicleStatus)
        if vehicleID in arrived && vehicleStatus[vehicleID].arrived==false
            vehicleStatus[vehicleID].arrived = true
            carhandler.isshowns[vehicleVisual[vehicleID]] = false
        end
        if !vehicleStatus[vehicleID].arrived
            #vehicles still running
            speed = traci.vehicle[:getSpeed](vehicleID)
            pos = traci.vehicle[:getPosition](vehicleID)
            #edge = traci.vehicle[:getRoadID](vehicleID)
            #push!(vehicleStatus[vehicleID].edge, edge)
            x = pos[1]*scalex-5
            y = pos[2]*scaley+45
            if (carhandler.xs[vehicleVisual[vehicleID]]-x)^2 + (carhandler.ys[vehicleVisual[vehicleID]]-y)^2 > 25
                #cars[:update_geom](vehicleVisual[vehicleID], x, y, 1)
                update_geom(carhandler, vehicleVisual[vehicleID], pos[1], pos[2], 1)
                #cars[:handler] = carhandler
                #vehicleVisual[vehicleID][:x] = x
                #vehicleVisual[vehicleID][:y] = y
            end
        end
    end
    #=
    for (vehicleID, start, dest, edge, pos, speed) in moveNodes
        #_checkInitialPositions(vehicleID, start, dest, edge, pos, time, speed, viewer, cars, carhandler)
        if vehicleID in keys(vehicleStatus)
            # make sure this is the studied vehicle
            #push!(vehicleStatus[vehicleID].step, Int(time/1000))
            #push!(vehicleStatus[vehicleID].edge, edge)
            x = pos[1]*scale-5
            y = pos[2]*scale+45
            if (carhandler.xs[vehicleVisual[vehicleID]]-x)^2 + (carhandler.ys[vehicleVisual[vehicleID]]-y)^2 > 25
                #cars[:update_geom](vehicleVisual[vehicleID], x, y, 1)
                update_geom(carhandler, vehicleVisual[vehicleID], pos[1], pos[2], 1)
                #cars[:handler] = carhandler
                #vehicleVisual[vehicleID][:x] = x
                #vehicleVisual[vehicleID][:y] = y
            end
        end
    end

    for vehicleID in keys(vehicleStatus)
        if vehicleID in arrived && vehicleStatus[vehicleID].arrived==false
            vehicleStatus[vehicleID].arrived = true
            carhandler.isshowns[vehicleVisual[vehicleID]] = false
            #assert(cars[:handler].isshowns[vehicleVisual[vehicleID]] == false)
            #vehicleVisual[vehicleID][:isshown] = false
        end
    end
    =#
end
function main(T)
 sumoExe = SUMO
 sumoConfig = SUMO_CONFIG
 traci.start([sumoExe, "-c", sumoConfig])
 traci.simulation[:subscribe](varIDs=(tc.VAR_DEPARTED_VEHICLES_IDS, tc.VAR_ARRIVED_VEHICLES_IDS))
 viewer = Viewer(screen_width, screen_height)
 sideviewer = Viewer(side_screen_width,side_screen_height, 1000, 0)
 ymargin = 40
 xmargin = 20
 plotwidth = 760
 plotheight = 200
 datas = Dict{String, Any}()
 datas["bar1"] = [[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
 datas["bar2"] = zeros(Int, 14)
 datas["bar3"] = zeros(14)
 datas["bar3count"] = ones(14)*0.000001
 bar1 = BarPlot(plotwidth,plotheight,xmargin,ymargin)
 bar1[:set_axis](["Zone ID", "Loc Count"])
 bar1[:add_data](datas["bar1"], color = COLORPOOL, renew=true)
 bar2 = BarPlot(plotwidth,plotheight,xmargin,2*ymargin+plotheight)
 bar2[:set_axis](["Zone ID", "Veh Count"])
 bar2[:add_data](datas["bar2"], color = COLORPOOL, renew=true)
 bar3 = BarPlot(plotwidth,plotheight,xmargin,3*ymargin+2*plotheight)
 bar3[:add_data](datas["bar3"]./datas["bar3count"], color = COLORPOOL, renew=true)
 bar3[:set_axis](["Zone ID", "Travel Speed"])
 bar4 = BarPlot(plotwidth,plotheight,xmargin,4*ymargin+3*plotheight)
 bar4[:set_axis](["Zone ID", "Energy Cost Rate"])

 sideviewer[:add_geom](bar1)
 sideviewer[:add_geom](bar2)
 sideviewer[:add_geom](bar3)
 sideviewer[:add_geom](bar4)
 background = Image(BACKGROUND_FILE, screen_width, screen_height)
 viewer[:add_geom](background)

 carhandler = CarHandler(Float64[], Float64[], Int[], Bool[])
 cars = ImageCompound(CARICONS, 20, 20, carhandler)

 viewer[:add_geom](cars)
 for e in EDGES
  netStatus[e[:getID]()] = EdgeStatus(Int[],Float64[],Float64[],e[:getSpeed]())
 end
 #traci.vehicle[:subscribe]("1", (tc.VAR_ROAD_ID, tc.VAR_LANEPOSITION))
 t = 0
 while t<T
  #println(t)
  doStep(viewer, cars, carhandler, datas)
  viewer[:render]()
  bar1[:add_data](datas["bar1"])
  bar2[:add_data](datas["bar2"])
  bar3[:add_data](datas["bar3"]./datas["bar3count"])
  sideviewer[:render]()

  t=t+1
  if length(vehicleStatus) == NUMSTUDY
   simulationend = true
   for vehID in keys(vehicleStatus)
    simulationend = simulationend && vehicleStatus[vehID].arrived
   end
   if simulationend
    break
   end
  end

 end

 traci.close()
 viewer[:close]()
 sideviewer[:close]()
end
