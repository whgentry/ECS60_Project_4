#include "MapRouter.h"
#include "XMLParser.h"
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <algorithm>

#define DEGREES_TO_RADIANS(angle)   (M_PI * (angle) / 180.0)

class CMapRouter::CImplementation : public CXMLParser{
    public:
        class CNode;
        class CEdge;
        class CWay;
        class CWeightID;
        class CCompWeight;
        class CSavedPath;
        
        std::unordered_map<TNodeID, std::shared_ptr<CNode>> MNodes; // key=ul NodeID
        std::unordered_map<TNodeID, std::shared_ptr<CWay>> MWays; // key=ul WayID
        std::unordered_map<TNodeID, std::shared_ptr<CSavedPath>> MSavedDists; // key=ul Start node ID
        std::unordered_map<TNodeID, std::shared_ptr<CSavedPath>> MSavedTimes; // key=ul Start node ID
        
        std::string BType;
        TNodeID BID;
        std::unordered_map <std::string, std::string> BTags;
        std::vector< std::shared_ptr<CNode> > BNodes;
        
        //--- Parse Functions ---//
        virtual void StartElement(const std::string &name, const std::vector< TAttribute > &attrs){
            
            if (name.compare("node") == 0)
            {
                BType = name;
                BID = std::stoul(attrs[0].DValue);
                auto lat = std::stod(attrs[1].DValue);
                auto lon = std::stod(attrs[2].DValue);
                MNodes[BID] = std::make_shared<CNode>(BID,lat,lon);
                MSavedDists[BID] = std::make_shared<CSavedPath>(BID);
                MSavedTimes[BID] = std::make_shared<CSavedPath>(BID);
            }
            else if (name.compare("way") == 0)
            {
                BType = name;
                BID = std::stoul(attrs[0].DValue);
                MWays[BID] = std::make_shared<CWay>(BID);
            }
            else if (name.compare("tag") == 0 && BType.compare("way") == 0)
            {
                BTags[attrs[0].DValue] = attrs[1].DValue;
            }
            else if (name.compare("nd") == 0)
            {
                if (MNodes.find(std::stoul(attrs[0].DValue)) != MNodes.end())
                {
                    BNodes.push_back(MNodes[std::stoul(attrs[0].DValue)]);
                }
            }
            
            return;
        }
        virtual void EndElement(const std::string &name){
            if (name.compare("way") == 0)
            {
                auto Way_ptr = MWays[BID];
                // update Name, SpeedLimit_mph, and OneWay
                // Name
                if (!BTags["name"].empty())
                {
                    Way_ptr->Name = BTags["name"];
                }
                else if (!BTags["ref"].empty())
                {
                    Way_ptr->Name = BTags["ref"];
                }
                // SpeedLimit_mph
                if (!BTags["maxspeed"].empty())
                {
                    Way_ptr->SpeedLimit_mph = std::stod(BTags["maxspeed"]);
                }
                // OneWay
                if (BTags["oneway"].compare("yes") == 0)
                {
                    Way_ptr->OneWay = true;
                }
                    
                // iterate through nodes in Way_ptr and add them to nodes connected list
                for (unsigned int i = 1; i < BNodes.size(); i++)
                {
                    auto node0 = BNodes[i-1];
                    auto node1 = BNodes[i];
                    
                    // calculate distance and time
                    double distance = CMapRouter::HaversineDistance(node0->lat,node0->lon,node1->lat,node1->lon);
                    double time = distance / Way_ptr->SpeedLimit_mph;
                    
                    // Insert Edge into edge map for first node
                    node0->MEdges[node1->ID] = std::make_shared<CEdge>(node1,distance,time);
                    // assigne Way pointer in edge to current way
                    node0->MEdges[node1->ID]->DWay = Way_ptr;
                    // if not OneWay, make connection bidirectional
                    if (!Way_ptr->OneWay)
                    {
                        // repeat above steps for other direction
                        node1->MEdges[node0->ID] = std::make_shared<CEdge>(node0,distance,time);
                        node1->MEdges[node0->ID]->DWay = Way_ptr;
                    }
                }
                BTags.clear();
                BNodes.clear();
            }
            return;
        }
        
        //--- Load Functions ---//
        static double MaxDeltaLongitude(double dist, double lat){
            const double EarthCircumferenceMiles = 3959.88 * M_PI * 2.0;  
            return 360.0 * dist / (cos(DEGREES_TO_RADIANS(lat)) * EarthCircumferenceMiles);
        };
        static double MaxDeltaLatitude(double dist){
            const double EarthCircumferenceMiles = 3959.88 * M_PI * 2.0;  
            return 360.0 * dist / EarthCircumferenceMiles;
        };
        
        //--- Path Algorithms ---//
        enum SearchType {shortest, fastest};
        class CWeightID {
            public:
                double Weight;
                TNodeID ID;
                TNodeID Previous;
                CWeightID(double w, TNodeID i) : Weight(w), ID(i) {
                }
        };
        class CCompWeight{
            public:
                bool operator()(const std::shared_ptr<CWeightID> &wid0, const std::shared_ptr<CWeightID> &wid1)
                {
                    if (wid0->Weight > wid1->Weight)
                        return true;
                    
                    return false; 
                }
                CCompWeight() = default;
                ~CCompWeight() = default;
        };
        class CSavedPath{
            public:
                std::unordered_set<TNodeID> Visited;
                std::unordered_map<TNodeID, std::shared_ptr<CWeightID>> MWeights;
                std::vector< std::shared_ptr<CWeightID> > HWeights;
                
                CSavedPath(TNodeID src){
                    //Visited.insert(src);
                    MWeights[src] = std::make_shared<CWeightID>(0.0,src);
                    MWeights[src]->Previous = src;
                    HWeights.push_back(MWeights[src]);
                }
        };
        static double EdgeWeight(const std::shared_ptr<CEdge> &edge, const SearchType &type){
            switch(type)
            {
                case shortest:
                    return edge->distance;
                    break;
                case fastest:
                    return edge->time;
                    break;
            }
            return 0.0;
        }
        double Dijkstra(const TNodeID &src, const TNodeID &dest, std::vector< TNodeID > &path, const SearchType &type){
            // https://www.youtube.com/watch?v=gdmfOwyQlcI
            // Setup
            //std::cout<<"Dijkstra Starting..."<<std::endl;
            auto src_ptr = MNodes[src];
            auto dest_ptr = MNodes[dest];
            path.clear();
            std::shared_ptr<CSavedPath> P;
            // load correct type (time or distance)
            switch(type)
            {
                case shortest: P = MSavedDists[src]; break;
                case fastest:  P = MSavedTimes[src]; break;
            }
            
            if (P->Visited.find(dest) == P->Visited.end())
            {
                // create Comp function object with correct type comparison
                CCompWeight DCompWeight;
                
                do
                {   // if heap is empty, path is not possible
                    if (P->HWeights.empty())
                    {
                        //std::cout<<"...Finished"<<std::endl;
                        return std::numeric_limits<double>::max();
                    }
                    // take smallest tentative distance and remove from heap and add to visited
                    std::pop_heap(P->HWeights.begin(),P->HWeights.end(),DCompWeight);
                    P->Visited.insert(P->HWeights.back()->ID);
                    src_ptr = MNodes[P->HWeights.back()->ID];
                    P->HWeights.pop_back();
                    // assign weights to connected nodes
                    for (auto &pair : src_ptr->MEdges)
                    {
                        auto con_ID = pair.second->DNode.lock()->ID;
                        auto src_ID = src_ptr->ID;
                        // if not visited
                        if (P->Visited.find(con_ID) == P->Visited.end())
                        {   // if weight is infinity 
                            if (P->MWeights.find(con_ID) == P->MWeights.end())
                            {
                                P->MWeights[con_ID] = std::make_shared<CWeightID>(P->MWeights[src_ID]->Weight + EdgeWeight(pair.second,type),con_ID);
                                P->MWeights[con_ID]->Previous = src_ID;
                                P->HWeights.push_back(P->MWeights[con_ID]);
                                std::push_heap(P->HWeights.begin(),P->HWeights.end(),DCompWeight);
                            }
                            else
                            {   // if current weight is greater then speculative weight
                                if( P->MWeights[con_ID]->Weight > (P->MWeights[src_ID]->Weight + EdgeWeight(pair.second,type)) )
                                {
                                    P->MWeights[con_ID]->Weight = P->MWeights[src_ID]->Weight + EdgeWeight(pair.second,type);
                                    P->MWeights[con_ID]->Previous = src_ID;
                                    std::make_heap(P->HWeights.begin(),P->HWeights.end(),DCompWeight);
                                }
                            }
                        }
                    }
                    //std::cout<<"...after for loop..."<<std::endl;
                } while (src_ptr != dest_ptr);
            }
            
            // assign the path and return weight (distance or time)
            //std::cout<<"...out of loop..."<<std::endl;
            auto pathFinder = P->MWeights[dest];
            while (pathFinder->Previous != pathFinder->ID)
            {
                path.push_back(pathFinder->ID);
                pathFinder = P->MWeights[pathFinder->Previous];
            }
            path.push_back(src);
            std::reverse(path.begin(),path.end());
            //std::cout<<"...Finished"<<std::endl;
            return P->MWeights[dest]->Weight;
        }

        //--- Class Definition ---//
        class CNode{
            public:
                CMapRouter::TNodeID ID;
                double lat;
                double lon;
                std::unordered_map< TNodeID, std::shared_ptr<CEdge> > MEdges;
                
                CNode() : ID(0.0), lat(0.0), lon(0.0) {}
                CNode(TNodeID d) : ID(d), lat(0.0), lon(0.0) {}
                CNode(TNodeID d, double la, double lo) : ID(d), lat(la), lon(lo) {}
                ~CNode() = default;
        };
        class CEdge{
            public:
                double distance;
                double time;
                std::shared_ptr<CWay> DWay;
                std::weak_ptr<CNode> DNode;
                                
                CEdge() = default;
                CEdge(const std::shared_ptr<CNode> &node) {
                    DNode = node;
                }
                CEdge(const std::shared_ptr<CNode> &node, double d, double t) {
                    DNode = node;
                    distance = d;
                    time = t;
                }
                ~CEdge() = default;
        };
        class CWay{
            public:
                CMapRouter::TNodeID ID;
                bool OneWay;
                std::string Name;
                double SpeedLimit_mph;
                                
                CWay() : OneWay(false), SpeedLimit_mph(25.0) {}
                CWay(TNodeID d) : ID(d), OneWay(false), SpeedLimit_mph(25.0) {}
                ~CWay() = default;
        };
        
        //--- Stuctors ---//
        CImplementation() = default;
        virtual ~CImplementation() = default;
};

CMapRouter::CMapRouter() : DData(std::make_unique< CImplementation>()){
    
}

CMapRouter::~CMapRouter(){
    
}


// Modified from https://rosettacode.org/wiki/Haversine_formula#C.2B.2B
double CMapRouter::HaversineDistance(double lat1, double lon1, double lat2, double lon2){
	double LatRad1 = DEGREES_TO_RADIANS(lat1);
	double LatRad2 = DEGREES_TO_RADIANS(lat2);
	double LonRad1 = DEGREES_TO_RADIANS(lon1);
	double LonRad2 = DEGREES_TO_RADIANS(lon2);
	double DeltaLat = LatRad2 - LatRad1;
	double DeltaLon = LonRad2 - LonRad1;
	double DeltaLatSin = sin(DeltaLat/2);
	double DeltaLonSin = sin(DeltaLon/2);	
	double Computation = asin(sqrt(DeltaLatSin * DeltaLatSin + cos(LatRad1) * cos(LatRad2) * DeltaLonSin * DeltaLonSin));
	const double EarthRadiusMiles = 3959.88;
	
	return 2 * EarthRadiusMiles * Computation;
}

void CMapRouter::GetMapExtents(double &minlat, double &minlon, double &maxlat, double &maxlon) const{
    minlat = DData->MNodes.begin()->second->lat;
    minlon = DData->MNodes.begin()->second->lon;
    maxlat = DData->MNodes.begin()->second->lat;
    maxlon = DData->MNodes.begin()->second->lon;
    
    for (const auto &mapPair : DData->MNodes)
    {
        if (minlat > mapPair.second->lat)
            minlat = mapPair.second->lat;
        if (minlon > mapPair.second->lon)
            minlon = mapPair.second->lon;
        if (maxlat < mapPair.second->lat)
            maxlat = mapPair.second->lat;
        if (maxlon < mapPair.second->lon)
            maxlon = mapPair.second->lon;
    }
    /*
    std::cout<< "minlat: "<< minlat << std::endl;
    std::cout<< "minlon: "<< minlon << std::endl;
    std::cout<< "maxlat: "<< maxlat << std::endl;
    std::cout<< "maxlon: "<< maxlon << std::endl;
    */
    return;
}

void CMapRouter::LoadMap(std::istream &is){
    DData->Parse(is);
    return;
}

CMapRouter::TNodeID CMapRouter::FindClosestNode(double lat, double lon){

    auto minDistNode = DData->MNodes.begin()->second;
    double minDist = HaversineDistance(minDistNode->lat,minDistNode->lon,lat,lon);
    
    for (const auto &mapPair : DData->MNodes)
    {
        double dist = HaversineDistance(mapPair.second->lat,mapPair.second->lon,lat,lon);
        if (dist < minDist)
        {
            minDist = dist;
            minDistNode = mapPair.second;
        }
    }
    
    return minDistNode->ID;
}

double CMapRouter::FindShortestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path){
    CImplementation::SearchType type = CImplementation::shortest;
    return DData->Dijkstra(src,dest,path,type);
}

double CMapRouter::FindFastestPath(TNodeID src, TNodeID dest, std::vector< TNodeID > &path){
    CImplementation::SearchType type = CImplementation::fastest;
    return DData->Dijkstra(src,dest,path,type);
}

bool CMapRouter::GetPathStreetNames(const std::vector< TNodeID > &path, std::vector< std::string > &streetnames) const{
    streetnames.clear();
    for (unsigned int i = 1; i < path.size(); i++)
    {
        TNodeID ID0 = path[i-1];
        TNodeID ID1 = path[i];
        auto name = DData->MNodes[ID0]->MEdges[ID1]->DWay->Name;
        if ( !name.empty() && (streetnames.empty() || name.compare(streetnames.back()) != 0) ) 
        {
            streetnames.push_back(name);
        }
    }
    return true;
}
