package com.example.superproject;

import javafx.geometry.Point2D;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class Utility_Functions {
    public static Point2D Cross(double c, final Point2D v){
        return new Point2D(-c * v.getY(), c * v.getX());
    }

    static List<Point> IntersectsPoints(Block block1, Block block2){
        List<Point> block1Points = block1.getPointList();
        List<Point> block2Points = block2.getPointList();
        List<Point> point2DS = new ArrayList<>();
        for (Point block1_point : block1Points) {
            if (block2.contains(block1_point)) {
                point2DS.add(block1_point);
            }
        }
        for(Point block2_point : block2Points){
            if(block1.contains(block2_point)){
                point2DS.add(block2_point);
            }
        }
        return point2DS;
    }
    static double tangentVelocity(Point2D velocity, Point2D radius){
        Point2D PerpendicularRadius = new Point2D(radius.getY()*-1,radius.getX());
        PerpendicularRadius = PerpendicularRadius.normalize();
        return velocity.dotProduct(PerpendicularRadius);
    }

    static final Comparator<Block> OSX = new Comparator<Block>() {
        @Override
        public int compare(Block one, Block two) {
            return Double.compare(two.getPointList().get(0).getPos().getX(), one.getPointList().get(0).getPos().getX());
        }
    };
    static final Comparator<Block> OSY = new Comparator<Block>() {
        @Override
        public int compare(Block one, Block two) {
            return Double.compare(two.getPointList().get(0).getPos().getY(), one.getPointList().get(0).getPos().getY());
        }
    };

    static void sortOSX(List<Block> blocks){
        blocks.sort(OSX);
    }

    static void sortOSY(List<Block> blocks){
        blocks.sort(OSY);
    }

}
