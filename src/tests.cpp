#include <gtest/gtest.h>
#include "MapRouter.h"
#include <sstream>
#include <iostream>

TEST(MapRouter, BasicTest){
    std::stringstream OSMStream("<?xml version='1.0' encoding='UTF-8'?>"
                                "<osm version=\"0.6\" generator=\"osmconvert 0.8.5\">"
                                "  <node id=\"0\" lat=\"38.0\" lon=\"-121.0\"/>"
                                "  <node id=\"1\" lat=\"39.0\" lon=\"-121.0\"/>"
                                "  <node id=\"2\" lat=\"38.0\" lon=\"-120.0\"/>"
                                "  <node id=\"3\" lat=\"39.0\" lon=\"-120.0\"/>"
                                "  <way id=\"0\">"
                                "    <nd ref=\"0\"/>"
                                "    <nd ref=\"1\"/>"
                                "    <nd ref=\"3\"/>"
                                "    <nd ref=\"2\"/>"
                                "    <tag k=\"highway\" v=\"residential\"/>"
                                "    <tag k=\"name\" v=\"Main Street\"/>"
                                "  </way>"
                                "  <way id=\"1\">"
                                "    <nd ref=\"0\"/>"
                                "    <nd ref=\"2\"/>"
                                "    <tag k=\"oneway\" v=\"yes\"/>"
                                "    <tag k=\"maxspeed\" v=\"15 mph\"/>"
                                "    <tag k=\"highway\" v=\"residential\"/>"
                                "    <tag k=\"name\" v=\"Shortcut Way\"/>"
                                "  </way>"    
                                "</osm>");
    CMapRouter Router;
    std::vector< CMapRouter::TNodeID > Path;
    std::vector< std::string > Streets;
    
    Router.LoadMap(OSMStream);
    EXPECT_EQ(Router.FindClosestNode(38.1,-121.0), 0);
    EXPECT_EQ(Router.FindClosestNode(38.1,-120.0), 2);
    EXPECT_EQ(Router.FindShortestPath(0,2,Path), 54.461481057457596933);
    EXPECT_EQ(Path.size(), 2);
    if(Path.size() == 2){
        EXPECT_EQ(Path[0],0);
        EXPECT_EQ(Path[1],2);
    }
    EXPECT_EQ(Router.FindFastestPath(0,2,Path), 3.6307654038305066102);
    EXPECT_EQ(Path.size(), 2);
    if(Path.size() == 2){
        EXPECT_EQ(Path[0],0);
        EXPECT_EQ(Path[1],2);
    }    
    EXPECT_TRUE(Router.GetPathStreetNames(Path,Streets));
    EXPECT_EQ(Streets.size(), 1);
    if(Path.size() == 1){
        EXPECT_EQ(Streets[0], "Shortcut Way");
    }
    EXPECT_EQ(Router.FindShortestPath(2,0,Path), 191.93646327377945227);
    EXPECT_EQ(Path.size(), 4);
    if(Path.size() == 4){
        EXPECT_EQ(Path[0],2);
        EXPECT_EQ(Path[1],3);
        EXPECT_EQ(Path[2],1);
        EXPECT_EQ(Path[3],0);
    }
    Streets.clear();
    EXPECT_TRUE(Router.GetPathStreetNames(Path,Streets));
    EXPECT_EQ(Streets.size(), 1);
    if(Path.size() == 1){
        EXPECT_EQ(Streets[0], "Main Street");
    }
}
