clc;
close all;
clear all;

% Define 2D grid map matrix
X_SCALE = 20;
Y_SCALE = 9;
MAP_2D = 2 * (ones(X_SCALE, Y_SCALE));

% Set the value of obstacle grid squares to -1, goal point to 0, starting point to 1, and passable point to 2
figure;
n = 0;
x_step = 1;
y_step = 1;
axis([1 X_SCALE+1 1 Y_SCALE+1]);
set(gca,'XTick',0:1:20);
set(gca,'YTick',0:1:9);
set(gcf,'Position',[200,200,1500,600], 'color','#E0FFFF')
daspect([1 1 1]);
grid on;
set(gca, 'GridColor', 'k','LineWidth',1.5,'GridAlpha',0.5,'GridColor','r'); 
hold on;
n = 0; 
pause(1);
text_msg = msgbox('Please select the goal point');
ht = findobj(text_msg, 'Type', 'text');
set(ht, 'FontSize', 12, 'Unit', 'normal');
uiwait(text_msg, 2);
if ishandle(text_msg) == 1
    delete(text_msg);
end
xlabel('Click the goal point using the left mouse button','Color','black','FontSize',15);
but = 0;
while (but ~= 1)
    [point_x, point_y, but] = ginput(1);
end
point_x = floor(point_x);
point_y = floor(point_y);
xTarget = point_x;  
yTarget = point_y;
MAP_2D(point_x, point_y) = 0;
plot(point_x + .5, point_y + .5, 'p', 'markersize', 8, 'LineWidth', 4, 'color', 'r');
text(point_x + 1, point_y + .5, 'Goal Point', 'FontSize', 15)

pause(2);
text_msg = msgbox('Select the obstacles');
ht = findobj(text_msg, 'Type', 'text');
set(ht, 'FontSize', 12, 'Unit', 'normal');
xlabel('Click the obstacle points using the left mouse button, and end the last obstacle point with the right mouse button','Color','black','FontSize',15);
uiwait(text_msg, 10);
if ishandle(text_msg) == 1
    delete(text_msg);
end
while but == 1
    [point_x, point_y, but] = ginput(1);
    point_x = floor(point_x);
    point_y = floor(point_y);
    MAP_2D(point_x, point_y) = -1;
    plot(point_x + .5, point_y + .5, 'x', 'markersize', 20, 'Color', 'k', 'LineWidth', 3);
end
 
pause(1);

text_msg = msgbox('Click the starting point with the left mouse button');
ht = findobj(text_msg, 'Type', 'text');
set(ht, 'FontSize', 12, 'Unit', 'normal');
uiwait(text_msg, 5);
if ishandle(text_msg) == 1
    delete(text_msg);
end
xlabel('Click the starting point using the left mouse button ','Color','black','FontSize',15);
but = 0;
while (but ~= 1)
    [point_x, point_y, but] = ginput(1);
    point_x = floor(point_x);
    point_y = floor(point_y);
end
xStart = point_x; 
yStart = point_y;
MAP_2D(point_x, point_y) = 1;
plot(point_x + .5, point_y + .5, 'bo', 'markersize', 15, 'Color', 'b', 'LineWidth', 3);
text(point_x - .5, point_y - .5, 'Starting Point', 'FontSize', 15)

OPEN_LIST = [];

CLOSED_LIST = [];
 
counter = 1; 
for m = 1:X_SCALE
    for n = 1:Y_SCALE
        if(MAP_2D(m, n) == -1)
            CLOSED_LIST(counter, 1) = m; 
            CLOSED_LIST(counter, 2) = n; 
            counter = counter + 1;
        end
    end
end
CLOSED_COUNT = size(CLOSED_LIST, 1);

xPonit = point_x;
yPoint = point_y;
OPEN_COUNT = 1;
Road_cost = 0;
goal_distance = distance(xPonit, yPoint, xTarget, yTarget);
% The distance function is used to calculate the distance, using diagonal distance, allowing movement in eight directions

OPEN_LIST(OPEN_COUNT,:) = openlist_insert(xPonit, yPoint, xPonit, yPoint, Road_cost, goal_distance, goal_distance);
OPEN_LIST(OPEN_COUNT, 1) = 0;
CLOSED_COUNT = CLOSED_COUNT + 1;
CLOSED_LIST(CLOSED_COUNT, 1) = xPonit;
CLOSED_LIST(CLOSED_COUNT, 2) = yPoint;
NoPath = 1;

% A* algorithm
while((xPonit ~= xTarget || yPoint ~= yTarget) && NoPath == 1)
 expand_map = Expand_function(xPonit, yPoint, Road_cost, xTarget, yTarget, CLOSED_LIST, X_SCALE, Y_SCALE);
 exp_count = size(expand_map, 1);
 for m = 1:exp_count
    flag = 0;
    for n = 1:OPEN_COUNT
        if(expand_map(m, 1) == OPEN_LIST(n, 2) && expand_map(m, 2) == OPEN_LIST(n, 3) )
            OPEN_LIST(n, 8) = min(OPEN_LIST(n, 8), expand_map(m, 5)); 
            if OPEN_LIST(n, 8) == expand_map(m, 5)
                OPEN_LIST(n, 4) = xPonit;
                OPEN_LIST(n, 5) = yPoint;
                OPEN_LIST(n, 6) = expand_map(m, 3);
                OPEN_LIST(n, 7) = expand_map(m, 4);
            end;
            flag = 1;
        end;

    end;
    if flag == 0
        OPEN_COUNT = OPEN_COUNT + 1;
        OPEN_LIST(OPEN_COUNT, :) = openlist_insert(expand_map(m, 1), expand_map(m, 2), xPonit, yPoint, expand_map(m, 3), expand_map(m, 4), expand_map(m, 5));
     end;   
 end;%

 % Find the node with the minimum f(n)

  index_min_node = minimum_fn(OPEN_LIST, OPEN_COUNT, xTarget, yTarget);
  if (index_min_node ~= -1)
   xPonit = OPEN_LIST(index_min_node, 2);
   yPoint = OPEN_LIST(index_min_node, 3);
   Road_cost = OPEN_LIST(index_min_node, 6);

  CLOSED_COUNT = CLOSED_COUNT + 1;
  CLOSED_LIST(CLOSED_COUNT, 1) = xPonit;
  CLOSED_LIST(CLOSED_COUNT, 2) = yPoint;
  OPEN_LIST(index_min_node, 1) = 0;
  else
      NoPath = 0;
  end;
end; 

m = size(CLOSED_LIST, 1);
Optimal_path = [];
point_x = CLOSED_LIST(m, 1);
point_y = CLOSED_LIST(m, 2);
m = 1;
Optimal_path(m, 1) = point_x;
Optimal_path(m, 2) = point_y;
m = m + 1;

if ((point_x == xTarget) && (point_y == yTarget))
    inode = 0;
   parent_x = OPEN_LIST(return_node_index(OPEN_LIST, point_x, point_y), 4);
   parent_y = OPEN_LIST(return_node_index(OPEN_LIST, point_x, point_y), 5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(m, 1) = parent_x;
           Optimal_path(m, 2) = parent_y;
           
           inode = return_node_index(OPEN_LIST, parent_x, parent_y);
           parent_x = OPEN_LIST(inode, 4);
           parent_y = OPEN_LIST(inode, 5);
           m = m + 1;
    end;
 n = size(Optimal_path, 1);
 % Draw the path 
 road = plot(Optimal_path(n, 1) + .5, Optimal_path(n, 2) + .5, 'bo', 'markersize', 15, 'Color', 'b', 'LineWidth', 3);
 n = n - 1;
 for m = n:-1:1
  pause(.25);
  set(road, 'XData', Optimal_path(m, 1) + .5, 'YData', Optimal_path(m, 2) + .5);
 drawnow ;
 end;
 Optimal_path = [Optimal_path; xStart, yStart]
 plot(Optimal_path(:, 1) + .5, Optimal_path(:, 2) + .5, 'LineWidth', 3, 'Color', 'k');
else
 pause(1);
 text_msg = msgbox('Unable to reach the goal point','warn');
 ht = findobj(text_msg, 'Type', 'text');
 set(ht, 'FontSize', 12, 'Unit', 'normal');
 uiwait(text_msg, 5);
end

function dist = distance(x1,y1,x2,y2)
 dist=sqrt((x1-x2)^2 + (y1-y2)^2);
end

function exp_array=Expand_function(node_x,node_y,hn,xTarget,yTarget,CLOSED,MAX_X,MAX_Y)    
    exp_array=[];
    exp_count=1;
    c2=size(CLOSED,1);
    for k= 1:-1:-1
        for j= 1:-1:-1
            if (k~=j || k~=0)
                s_x = node_x+k;
                s_y = node_y+j;
                if( (s_x >0 && s_x <=MAX_X) && (s_y >0 && s_y <=MAX_Y))
                    flag=1;                    
                    for c1=1:c2
                        if(s_x == CLOSED(c1,1) && s_y == CLOSED(c1,2))
                            flag=0;
                        end;
                    end;
                    if (flag == 1)
                        exp_array(exp_count,1) = s_x;
                        exp_array(exp_count,2) = s_y;
                        exp_array(exp_count,3) = hn+distance(node_x,node_y,s_x,s_y);
                        exp_array(exp_count,4) = distance(xTarget,yTarget,s_x,s_y);
                        exp_array(exp_count,5) = exp_array(exp_count,3)+exp_array(exp_count,4);
                        exp_count=exp_count+1;
                    end
                end
            end
        end
    end  
end



function [fitresult, gof] = createFit(x, y)
[xData, yData] = prepareCurveData( x, y );

% Set up fittype and options.
ft = 'splineinterp';

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, 'Normalize', 'on' );

% Plot fit with data.
figure( 'Name', 'untitled fit 1' );
h = plot( fitresult, xData, yData );
legend( h, 'y vs. x', 'untitled fit 1', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 'x', 'Interpreter', 'none' );
ylabel( 'y', 'Interpreter', 'none' );
grid on
end

function i_min = minimum_fn(OPEN,OPEN_COUNT,xTarget,yTarget)
 temp_array=[];
 k=1;
 flag=0;
 goal_index=0;
 for j=1:OPEN_COUNT
     if (OPEN(j,1)==1)
         temp_array(k,:)=[OPEN(j,:) j];
         if (OPEN(j,2)==xTarget && OPEN(j,3)==yTarget)
             flag=1;
             goal_index=j;
         end;
         k=k+1;
     end;
 end;
 if flag == 1
     i_min=goal_index;
 end
 if size(temp_array ~= 0)
  [minimum_fn,temp_min]=min(temp_array(:,8));
  i_min=temp_array(temp_min,9);
 else
     i_min=-1;
 end;

 function new_row = openlist_insert(xval,yval,parent_xval,parent_yval,hn,gn,fn)
new_row=[1,8];
new_row(1,1)=1;
new_row(1,2)=xval;
new_row(1,3)=yval;
new_row(1,4)=parent_xval;
new_row(1,5)=parent_yval;
new_row(1,6)=hn;
new_row(1,7)=gn;
new_row(1,8)=fn;

 end

function n_index = return_node_index(OPEN,xval,yval)
    i=1;
    while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )
        i=i+1;
    end;
    n_index=i;
end
end
