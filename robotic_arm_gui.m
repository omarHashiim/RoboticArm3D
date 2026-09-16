function robotic_arm_gui()

clc;
close all;



L1 = 3;
L2 = 4;
L3 = 3;
L4 = 1.5;



initialAngles = [30 35 -25 20];


fig = uifigure( ...
    'Name','3D Robotic Arm Simulator', ...
    'Position',[100 80 1200 750]);


ax = uiaxes(fig, ...
    'Position',[330 80 830 630]);

title(ax,'4-DOF INDUSTRIAL ROBOTIC ARM');

xlabel(ax,'X Axis');
ylabel(ax,'Y Axis');
zlabel(ax,'Z Axis');

grid(ax,'on');
axis(ax,'equal');

xlim(ax,[-11 11]);
ylim(ax,[-11 11]);
zlim(ax,[0 13]);

view(ax,45,30);


uilabel(fig, ...
    'Position',[30 650 100 25], ...
    'Text','Joint 1');

s1 = uislider(fig, ...
    'Position',[30 630 240 3], ...
    'Limits',[-180 180], ...
    'Value',initialAngles(1));


uilabel(fig, ...
    'Position',[30 545 100 25], ...
    'Text','Joint 2');

s2 = uislider(fig, ...
    'Position',[30 525 240 3], ...
    'Limits',[-90 90], ...
    'Value',initialAngles(2));


uilabel(fig, ...
    'Position',[30 440 100 25], ...
    'Text','Joint 3');

s3 = uislider(fig, ...
    'Position',[30 420 240 3], ...
    'Limits',[-90 90], ...
    'Value',initialAngles(3));


uilabel(fig, ...
    'Position',[30 335 100 25], ...
    'Text','Joint 4');

s4 = uislider(fig, ...
    'Position',[30 315 240 3], ...
    'Limits',[-90 90], ...
    'Value',initialAngles(4));


uilabel(fig, ...
    'Position',[30 230 100 25], ...
    'Text','Gripper');

gripperSlider = uislider(fig, ...
    'Position',[30 210 240 3], ...
    'Limits',[0 100], ...
    'Value',50);


uibutton(fig, ...
    'Position',[30 140 105 42], ...
    'Text','RESET', ...
    'ButtonPushedFcn',@resetRobot);


uibutton(fig, ...
    'Position',[165 140 105 42], ...
    'Text','ANIMATE', ...
    'ButtonPushedFcn',@animateRobot);


positionLabel = uilabel(fig, ...
    'Position',[30 55 270 60], ...
    'Text','End Effector Position');

 


s1.ValueChangedFcn = @updateRobot;
s2.ValueChangedFcn = @updateRobot;
s3.ValueChangedFcn = @updateRobot;
s4.ValueChangedFcn = @updateRobot;

gripperSlider.ValueChangedFcn = @updateRobot;



drawRobot(initialAngles,50);



function updateRobot(~,~)

    theta = [ ...
        s1.Value ...
        s2.Value ...
        s3.Value ...
        s4.Value];

    drawRobot(theta,gripperSlider.Value);

end



function resetRobot(~,~)

    s1.Value = 30;
    s2.Value = 35;
    s3.Value = -25;
    s4.Value = 20;

    gripperSlider.Value = 50;

    drawRobot([30 35 -25 20],50);

end



function animateRobot(~,~)

    startAngles = [30 35 -25 20];

    targetAngles = [-60 50 -45 60];

    steps = 100;

    trajectory = zeros(3,steps);

    for k = 1:steps

        t = k / steps;

        currentAngles = ...
            startAngles + ...
            (targetAngles-startAngles)*t;

        s1.Value = currentAngles(1);
        s2.Value = currentAngles(2);
        s3.Value = currentAngles(3);
        s4.Value = currentAngles(4);


        t1 = deg2rad(currentAngles(1));
        t2 = deg2rad(currentAngles(2));
        t3 = deg2rad(currentAngles(3));
        t4 = deg2rad(currentAngles(4));

        r2 = L2*cos(t2);

        r3 = r2 + ...
             L3*cos(t2+t3);

        r4 = r3 + ...
             L4*cos(t2+t3+t4);

        z4 = L1 + ...
             L2*sin(t2) + ...
             L3*sin(t2+t3) + ...
             L4*sin(t2+t3+t4);

        x4 = r4*cos(t1);
        y4 = r4*sin(t1);

        trajectory(:,k) = [x4;y4;z4];


        drawRobot( ...
            currentAngles, ...
            gripperSlider.Value);


        hold(ax,'on');

        plot3(ax, ...
            trajectory(1,1:k), ...
            trajectory(2,1:k), ...
            trajectory(3,1:k), ...
            'LineWidth',3);

        hold(ax,'off');

        drawnow;

        pause(0.025);

    end

end


function drawRobot(theta,gripperValue)

    cla(ax);


    t1 = deg2rad(theta(1));
    t2 = deg2rad(theta(2));
    t3 = deg2rad(theta(3));
    t4 = deg2rad(theta(4));

    

    P0 = [0;0;0];

    P1 = [0;0;L1];

    r2 = L2*cos(t2);

    z2 = L1 + ...
         L2*sin(t2);

    P2 = [ ...
        r2*cos(t1);
        r2*sin(t1);
        z2];

    r3 = r2 + ...
         L3*cos(t2+t3);

    z3 = L1 + ...
         L2*sin(t2) + ...
         L3*sin(t2+t3);

    P3 = [ ...
        r3*cos(t1);
        r3*sin(t1);
        z3];

    r4 = r3 + ...
         L4*cos(t2+t3+t4);

    z4 = L1 + ...
         L2*sin(t2) + ...
         L3*sin(t2+t3) + ...
         L4*sin(t2+t3+t4);

    P4 = [ ...
        r4*cos(t1);
        r4*sin(t1);
        z4];


    [Xg,Yg] = meshgrid(-10:1:10,-10:1:10);

    surf(ax, ...
        Xg,Yg,zeros(size(Xg)), ...
        'FaceAlpha',0.10, ...
        'EdgeColor',[0.7 0.7 0.7]);

    hold(ax,'on');


    [Xb,Yb,Zb] = cylinder(2.2,60);

    Zb = Zb * 0.8;

    surf(ax,Xb,Yb,Zb, ...
        'FaceAlpha',0.95, ...
        'EdgeColor','none');


    [Xt,Yt,Zt] = cylinder(1.7,60);

    Zt = Zt*0.4 + 0.8;

    surf(ax,Xt,Yt,Zt, ...
        'FaceAlpha',0.95, ...
        'EdgeColor','none');

     

    [Xc,Yc,Zc] = cylinder(1.15,50);

    Zc = Zc*(L1-0.8) + 0.8;

    surf(ax,Xc,Yc,Zc, ...
        'FaceAlpha',0.95, ...
        'EdgeColor','none');


    drawCylinderBetween( ...
        P1,P2,0.65);


    drawCylinderBetween( ...
        P2,P3,0.55);


    drawCylinderBetween( ...
        P3,P4,0.40);


    drawJoint(P1,0.9);


    drawJoint(P2,0.75);


    drawJoint(P3,0.65);


    drawJoint(P4,0.55);


    opening = ...
        0.25 + ...
        (gripperValue/100)*1.4;

    fingerLength = 1.0;

    leftStart = P4 + [0;opening/2;0];

    rightStart = P4 + [0;-opening/2;0];

    leftEnd = leftStart + ...
        [fingerLength;0;-0.7];

    rightEnd = rightStart + ...
        [fingerLength;0;-0.7];

    drawCylinderBetween( ...
        leftStart,leftEnd,0.16);

    drawCylinderBetween( ...
        rightStart,rightEnd,0.16);


    drawJoint(leftEnd,0.20);

    drawJoint(rightEnd,0.20);


    scatter3(ax, ...
        P4(1),P4(2),P4(3), ...
        100,'filled');


    quiver3(ax, ...
        0,0,0, ...
        3,0,0, ...
        'LineWidth',2);

    quiver3(ax, ...
        0,0,0, ...
        0,3,0, ...
        'LineWidth',2);

    quiver3(ax, ...
        0,0,0, ...
        0,0,3, ...
        'LineWidth',2);

    text(ax,3,0,0,' X');

    text(ax,0,3,0,' Y');

    text(ax,0,0,3,' Z');


    grid(ax,'on');

    axis(ax,'equal');

    xlim(ax,[-11 11]);

    ylim(ax,[-11 11]);

    zlim(ax,[0 13]);

    view(ax,45,30);

    title(ax,'4-DOF INDUSTRIAL ROBOTIC ARM');

    xlabel(ax,'X Axis');

    ylabel(ax,'Y Axis');

    zlabel(ax,'Z Axis');


    positionLabel.Text = sprintf( ...
        ['End Effector Position\n' ...
         'X = %.2f    Y = %.2f    Z = %.2f'], ...
         P4(1), ...
         P4(2), ...
         P4(3));

    hold(ax,'off');

end


function drawCylinderBetween(Pa,Pb,radius)

    direction = Pb-Pa;

    lengthLink = norm(direction);

    if lengthLink == 0
        return;
    end

    [X,Y,Z] = cylinder(radius,30);

    Z = Z * lengthLink;

    v = direction / lengthLink;

    zAxis = [0;0;1];

    rotationAxis = cross(zAxis,v);

    rotationAngle = acos(dot(zAxis,v));

    if norm(rotationAxis) > 1e-10

        rotationAxis = ...
            rotationAxis / norm(rotationAxis);

        K = [ ...
            0 -rotationAxis(3) rotationAxis(2);
            rotationAxis(3) 0 -rotationAxis(1);
            -rotationAxis(2) rotationAxis(1) 0];

        R = eye(3) + ...
            sin(rotationAngle)*K + ...
            (1-cos(rotationAngle))*K*K;

    else

        if dot(zAxis,v) < 0
            R = [1 0 0;0 -1 0;0 0 -1];
        else
            R = eye(3);
        end

    end

    points = [X(:)';Y(:)';Z(:)'];

    rotated = R*points;

    Xr = reshape(rotated(1,:),size(X)) + Pa(1);
    Yr = reshape(rotated(2,:),size(Y)) + Pa(2);
    Zr = reshape(rotated(3,:),size(Z)) + Pa(3);

    surf(ax, ...
        Xr,Yr,Zr, ...
        'FaceAlpha',0.95, ...
        'EdgeColor','none');

end


function drawJoint(P,radius)

    [X,Y,Z] = sphere(25);

    X = X*radius + P(1);

    Y = Y*radius + P(2);

    Z = Z*radius + P(3);

    surf(ax, ...
        X,Y,Z, ...
        'FaceAlpha',1, ...
        'EdgeColor','none');

end

end