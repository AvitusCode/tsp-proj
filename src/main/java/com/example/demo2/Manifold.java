package com.example.demo2;

import javafx.geometry.Point2D;
import javafx.scene.shape.Rectangle;
import javafx.scene.transform.Transform;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Manifold {
    private Block A;
    private Block B;

    public Manifold(Block A, Block B){
        this.A = A;
        this.B = B;

        solveCollision();
    }

    private void shapeOverlapDIAG(Block lhs, Block rhs){
        List<Point2D> poly1 = Utility_Functions.getPoints(lhs);
        List<Point2D> poly2 = Utility_Functions.getPoints(rhs);
        contacts = new ArrayList<>();

        // Нужно проверить обе фигуры
        for (int shape = 0; shape < 2; shape++) {

            if (shape == 1) {
                List<Point2D> temp = poly1;
                poly1 = poly2;
                poly2 = temp;
            }

            for (int p = 0; p < poly1.size(); p++) {
                Point2D lineR1s = shape == 0 ? Utility_Functions.CenterRectangle(lhs.getRectangle()) : Utility_Functions.CenterRectangle(rhs.getRectangle());
                Point2D lineR1e = poly1.get(p);
                Point2D depth = new Point2D(0, 0);

                for (int q = 0; q < poly2.size(); q++) {
                    Point2D lineR2s = poly2.get(q);
                    Point2D lineR2e = poly2.get((q + 1) % poly2.size());

                    // Standard "off the shelf" line intersection
                    double h = (lineR2e.getX() - lineR2s.getX()) * (lineR1s.getY() - lineR1e.getY()) -
                            (lineR1s.getX() - lineR1e.getX()) * (lineR2e.getY() - lineR2s.getY());
                    double t1 = ((lineR2s.getY() - lineR2e.getY()) * (lineR1s.getX() - lineR2s.getX()) +
                            (lineR2e.getX() - lineR2s.getX()) * (lineR1s.getY() - lineR2s.getY())) / h;
                    double t2 = ((lineR1s.getY() - lineR1e.getY()) * (lineR1s.getX() - lineR2s.getX()) +
                            (lineR1e.getX() - lineR1s.getX()) * (lineR1s.getY() - lineR2s.getY())) / h;


                    if (t1 >= 0.0 && t1 < 1.0 && t2 >= 0 && t2 < 1.0) {
                        depth = new Point2D(
                                depth.getX() + (1 - t1) * (lineR1e.getX() - lineR1s.getX()),
                                depth.getY() + (1 - t1) * (lineR1e.getY() - lineR1s.getY()));

                        if (shape == 0){
                            normal = Vec2.getNormal(q);
                            Transform transform = B.getRectangle().getLocalToParentTransform();
                            normal = transform.transform(normal);
                        }
                        else{
                            if (normal != null){
                                normal = normal.multiply(-1);
                            }
                            else{
                                normal = Vec2.getNormal(q);
                                Transform transform = A.getRectangle().getLocalToParentTransform();
                                normal = transform.transform(normal);
                            }
                        }

                        contacts.add(lineR1e);
                    }
                }

                // re-estimate pos
                depth  = depth.multiply(shape == 0 ? -1 : 1);
                lhs.physics_model.setPosition(new Point2D(lhs.getRectangle().getX() + depth.getX(), lhs.getRectangle().getY() + depth.getY()));
            }
        }

        normal = normal.multiply(-1);
        lhs.physics_model.contacts = contacts;
    }

    private void solveCollision(){ // Generate contact information
        shapeOverlapDIAG(A, B);
    }

    public void applyImpulse(){  // solve impulse
        Rectangle rA = A.getRectangle();
        Rectangle rB = B.getRectangle();

        // We need coordinates of center mass
        Point2D centerA = Utility_Functions.CenterRectangle(rA);
        Point2D centerB = Utility_Functions.CenterRectangle(rB);

       for (Point2D contact : contacts){
           Point2D RA = contact.subtract(centerA);
           Point2D RB = contact.subtract(centerB);

           // Разрешающая скорость
           Point2D RV = getResultSpeed(RA, RB);
           double RVContact = RV.dotProduct(normal);
           // Если положительно, то точки удаляются, либо движение сонаправлено
           if (RVContact > 0) {
               return;
           }

           double RACrossN = RA.getX() * normal.getY() - RA.getY() * normal.getX();
           double RBCrossN = RB.getX() * normal.getY() - RB.getY() * normal.getX();

           double iMassSum = 1.0 / A.physics_model.mass + 1.0 / B.physics_model.mass
                   + RACrossN * RACrossN * (1.0 / A.physics_model.inertia)
                   + RBCrossN * RBCrossN * (1.0 / B.physics_model.inertia);

           double j = -2.0 * RVContact / iMassSum;
           j /= contacts.size();

           Point2D impulse = normal.multiply(j);
           A.physics_model.applyImpulse(impulse.multiply(-1.0), RA);
           B.physics_model.applyImpulse(impulse, RB);
       }
    }

    // correcting position after collision and impulse applying (Применять при необходимости)
    public void posCorrection(){
        final double percent = 0.4; // [0.2; 0.8] to correction pos
        final double slop = 0.01;   // [0.01; 0.1] our epsilon
        final double iMassA =  1.0 / A.physics_model.mass;
        final double iMassB =  1.0 / B.physics_model.mass;

        double tempCalc = Math.max(0.0, displacement - slop) / (iMassA + iMassB) * percent;
        Point2D dist = normal.multiply(tempCalc);

        // исправляем позиции
        A.physics_model.setPosition(new Point2D(A.getRectangle().getX() - dist.multiply(iMassA).getX(), A.getRectangle().getY() - dist.multiply(iMassA).getY()));
        B.physics_model.setPosition(new Point2D(B.getRectangle().getX() - dist.multiply(iMassB).getX(), B.getRectangle().getY() - dist.multiply(iMassB).getY()));
    }

    private Point2D getResultSpeed(final Point2D RA, final Point2D RB){
        Point2D tempB = B.physics_model.velocity.add(Utility_Functions.Cross(B.physics_model.wVelocity, RB));
        Point2D tempA = A.physics_model.velocity.add(Utility_Functions.Cross(A.physics_model.wVelocity, RA));

        return tempB.subtract(tempA);
    }

    public Point2D normal;           // From A to B
    public List<Point2D> contacts;
    public double displacement;     // Depth of collision
}
