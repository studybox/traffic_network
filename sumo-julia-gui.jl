NUMSTUDY = 100
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
function _checkBackInitialPositions(vehicleID, start, dest, edge, pos, time, speed)
 if !(vehicleID in keys(backgroundStatus))
   backgroundStatus[vehicleID] = BackStatus(Int(time/1000), [edge], [speed], [Int(time/1000)], Int[], [speed], Float64[], Float64[], Float64[], dest==edge)
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

function doStep(viewer, cars, carhandler)
    traci.simulationStep()
    #moveNodes = []
    subscribes = traci.vehicle[:getSubscriptionResults]()
    time = traci.simulation[:getCurrentTime]()
    #=
    count = 0
    for (veh, subs) in subscribes
        if count <= 400
            route = traci.vehicle[:getRoute](veh)
            speed = traci.vehicle[:getSpeed](veh)
            pos = traci.vehicle[:getPosition](veh)
            #push!(moveNodes, (veh, route[1], route[end], subs[tc.VAR_ROAD_ID], pos, speed))
            count = count + 1
        end
    end
    =#
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
        _checkInitialPositions(v, start, dest, edge, pos, time, speed, viewer, cars, carhandler)
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
  doStep(viewer, cars, carhandler)
  viewer[:render]()
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
end
