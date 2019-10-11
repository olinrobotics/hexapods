% UI
close all;
plotAngles(7.23, -7.67, -8.25, 1);
plotAngles(7.23, -7.67, -8.25, 2);
plotAngles(7.23, -7.67, -8.25, 3);
plotAngles(7.23, -7.67, -8.25, 4);
plotAngles(7.23, -7.67, -8.25, 5);
plotAngles(7.23, -7.67, -8.25, 6);
% f = figure;
% xSlider = uicontrol('style','slider', 'position',[30,30,200,20],...
%     'min',-10, 'max',10, 'Value',0);
% ySlider = uicontrol('style','slider', 'position',[30,10,200,20],...
%     'min',-10, 'max',10, 'Value',0);
% zSlider = uicontrol('style','slider', 'position',[260,30,200,20],...
%     'min',-5, 'max',5, 'Value',0);
% legSlider = uicontrol('style','slider', 'position',[260,10,200,20],...
%     'min',1, 'max',6, 'Value',1, 'SliderStep',[1/6,1]);
% uicontrol('style','text', 'position',[10,30,20,20],'String','x');
% uicontrol('style','text', 'position',[10,10,20,20],'String','y');
% uicontrol('style','text', 'position',[240,30,20,20],'String','z');
% uicontrol('style','text', 'position',[240,10,20,20],'String','leg');
% xSlider.Callback = @callback;
% ySlider.Callback = @callback;
% zSlider.Callback = @callback;
% legSlider.Callback = @callback;
% guidata(f, [xSlider, ySlider, zSlider, legSlider]);

function callback(~,~)
    s = guidata(gcf);
    s(4).Value = round(s(4).Value);
    plotAngles(s(1).Value,s(2).Value,s(3).Value,s(4).Value);
end

% a = inner servo angle in radians
% b = middle servo angle in radians
% c = outer servo angle in radians
% x = distance forward in meters
% y = distance left in meters
% z = distance up in meters
% L1 = length of inner link in meters
% L2 = length of outer link in meters
% leg = index of leg
% R = distance from leg base to center in meters
function [a, b, c] = getAngles(x, y, z, L1, L2, L3, leg, R)
    b = 0;
    c = 0;
    x = x - R*cos(leg*pi/3 - pi/6);
    y = y - R*sin(leg*pi/3 - pi/6);
    a = atan2(y, x) + pi/6 - leg*pi/3
    r = sqrt(x^2+y^2) - L1;
    if a < -pi
        a = a + 2*pi;
    end
    if a < -pi/2
        a = a + pi;
        r = -sqrt(x^2+y^2) - L1;
    end
    if a > pi/2
        a = a - pi;
        r = -sqrt(x^2+y^2) - L1;
    end
    if r > L2 + L3 || r^2+z^2 == 0
        disp('Out of range');
        return
    end
    r1 = (r^3+r*z^2+r*L2^2-r*L3^2-z*sqrt(-r^4-z^4-(L2^2-L3^2)^2+2*z^2*...
        (L2^2+L3^2)+2*r^2*(-z^2+L2^2+L3^2)))/(2*(r^2+z^2));
    z1 = (z^3+z*r^2+z*L2^2-z*L3^2+r*sqrt(-r^4-z^4-(L2^2-L3^2)^2+2*z^2*...
        (L2^2+L3^2)+2*r^2*(-z^2+L2^2+L3^2)))/(2*(r^2+z^2));
    if imag(r1)^2 + imag(z1)^2 > 0
        disp('No valid solution');
        return
    end
    b = -atan2(z1, r1);
    c = -atan2(z-z1, r-r1) - b;
    
    minLimits = [0, 40, 0];
    maxLimits = [180, 180, 130];
    output = [180/pi*a + 90, -180/pi*b + 90, 180/pi*c]
    if sum((output < minLimits) + (output > maxLimits))
        disp('Out of bounds: ');
        disp((output < minLimits) + (output > maxLimits));
        return
    end
end

function plotAngles(x, y, z, leg)
    R = 5.35; % Distance from center to Servo A in inches
    L1 = 1.12; % Distance from Servo A to Servo B in inches
    L2 = 2.24; % Distance from Servo B to Servo C in inches
    L3 = 7; % Distance from Servo C to end effector in inches
    
    disp('Input');
    disp([x,y,z,leg]);
    [a, b, c] = getAngles(x, y, z, L1, L2, L3, leg, R);
    disp('Output');
    disp(rad2deg([a,b,c]))
    vertices = [1:6, 1];
    R0 = vrrotvec2mat([0,0,1,leg*pi/3-pi/6]);
    v0 = R0*[R;0;0];
    R1 = R0*vrrotvec2mat([0,0,1,a]);
    v1 = R1*[L1;0;0]+v0;
    R2 = R1*vrrotvec2mat([0,1,0,b]);
    v2 = R2*[L2;0;0]+v1;
    R3 = R2*vrrotvec2mat([0,1,0,c]);
    v3 = R3*[L3;0;0]+v2;
    v = [v0 v1 v2 v3];
    hold off;
    plot3(R*cos(vertices*pi/3-pi/6),R*sin(vertices*pi/3-pi/6),vertices*0);
    hold on; axis equal;
    plot3(v(1,:), v(2,:), v(3,:), 'k.-');
    plot3(v0(1),v0(2),v0(3),'r.','MarkerSize',20);
    plot3(x,y,z,'b.','MarkerSize',20);
end