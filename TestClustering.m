function output = TestClustering(pointCloudFileName)
rng (1);
multiplier = 1;
dispatcherRate=10; %10 FLSs per second

fileID=fopen(pointCloudFileName);

dispatchers=zeros(8,3);
dispatchers(1,:)=[0,0,0];
dispatchers(2,:)=[100,0,0];
dispatchers(3,:)=[100,0,100];
dispatchers(4,:)=[100,100,0];
dispatchers(5,:)=[0,0,100];
dispatchers(6,:)=[0,100,0];
dispatchers(7,:)=[0,100,100];
dispatchers(8,:)=[100,100,100];

% Read and discard the first two lines
currLine = textscan(fileID,'%s',1,'Delimiter','\n');
currLine = textscan(fileID,'%s',1,'Delimiter','\n');
currRow = char(currLine{1});
splittedRow = strsplit(currRow,' ');

splittedRow = str2double(splittedRow);
numVs = splittedRow(1);
vertices = [];
while (~feof(fileID))
    currLine = textscan(fileID,'%s',1,'Delimiter','\n');
    currRow = char(currLine{1});
    splittedRow = strsplit(currRow,' ');

    splittedRow = str2double(splittedRow);
    vertices = [vertices;[splittedRow(1)*multiplier, splittedRow(2)*multiplier, splittedRow(3)*multiplier]];
end
ptCloud = pointCloud([vertices]);

%pcshow(ptCloud)
%title('Point Cloud')
%[labels,numClusters] = pcsegdist(ptCloud,0.2);
%pcshow(ptCloud.Location,labels)
%colormap(hsv(numClusters))
%title('Point Cloud Clusters')

[idx,C] = kmeans(vertices,8);
%[~,idx] = pdist2(dispatchers,vertices,'euclidean','Smallest',1);
distdispatcherscentroid=zeros(8,8);
for v=1:size(C,1)
    for x=1:size(C,1)
        P = [C(v,[1,2,3]);dispatchers(x,[1,2,3])];
        d = pdist(P,'euclidean');
        distdispatcherscentroid(x,v) = d;
    end
end
dispatchercentroid = zeros(8,1);
for v=1:size(C,1)
    [M,I] = min(distdispatcherscentroid(v,:));
        if ismember(I,dispatchercentroid)
            [B,I1] = sort(distdispatcherscentroid(v,:));
            for u=1:size(C,1)
                if ~ismember(I1(u),dispatchercentroid)
                    dispatchercentroid(v,1)=I1(u);
                    break;
                end
            end
        else
            dispatchercentroid(v,1) = I;
        end
end
%datacluster = [[[vertices(idx==1,1),vertices(idx==1,2), vertices(idx==1,3)]]; [[vertices(idx==2,1),vertices(idx==2,2), vertices(idx==2,3)]]; [[vertices(idx==3,1),vertices(idx==3,2), vertices(idx==3,3)]]; [[vertices(idx==4,1),vertices(idx==4,2), vertices(idx==4,3)]]; [[vertices(idx==5,1),vertices(idx==5,2), vertices(idx==5,3)]]; [[vertices(idx==6,1),vertices(idx==6,2), vertices(idx==6,3)]]; [[vertices(idx==7,1),vertices(idx==7,2), vertices(idx==7,3)]]; [[vertices(idx==8,1),vertices(idx==8,2), vertices(idx==8,3)]];];
%disp(size(datacluster));
%distVertices = containers.Map();
dist = 0;
points=zeros(8,1);
pointsDispatcher = containers.Map("KeyType",'int32',"ValueType",'any');
distanceDispatcher = containers.Map("KeyType",'int32',"ValueType",'any');
noOfFls = [];
flightPathData = containers.Map("KeyType",'int32',"ValueType",'any');
count = 1;
comparisons=0;
for i=1:size(C,1)
    c = dispatchercentroid(i,1);
    var1 =[vertices(idx==c,1),vertices(idx==c,2), vertices(idx==c,3)];
    distDictionary = containers.Map("KeyType",'double',"ValueType",'int64');
    pointsDictionary = containers.Map("KeyType",'int64',"ValueType",'any');
    %count = 1;
    for j=1:size(var1,1)
        points(i)=points(i)+1;
        dist = dist + pdist([dispatchers(i,:);var1(j,:)],'euclidean');
        flightPathData(count) = [dispatchers(i,:)];
        pointsDictionary(count) = [var1(j,:)];
        distDictionary(count) = pdist([dispatchers(i,:);var1(j,:)],'euclidean');
        count = count +1;
    end
    pointsDispatcher(i) = pointsDictionary;
    distanceDispatcher(i) = distDictionary;
    noOfFls = [noOfFls;size(distanceDispatcher(i),1)];
end 
flsSpeed = 50;
arrived = 0;
flyFLS = [];
noOfFls1 = noOfFls;
flyingFLS = containers.Map("KeyType",'int32',"ValueType",'any');
cnt = 0;
timeLook = 0;
collisions = 0;
while arrived < size(vertices,1)
    if size(flyingFLS,1)>0
        flyFLS = [];
        for i=flyingFLS.keys
            i = cell2mat(i);
            v = flyingFLS(i);
            v1 = dispatchers(v(1),:);
            v2 = pointsDispatcher(v(1));
            v2 = v2(i);
            t = v(2);
            x = [1,0,0];
            y = [0,1,0];
            z = [0,0,1];
            theta_x = atan2d(norm(cross(v2,x)),dot(v2,x));
            theta_y = atan2d(norm(cross(v2,y)),dot(v2,y));
            theta_z = atan2d(norm(cross(v2,z)),dot(v2,z));
            velocity_x = flsSpeed*cosd(theta_x);
            velocity_y = flsSpeed*cosd(theta_y);
            velocity_z = flsSpeed*cosd(theta_z);

            new_x = velocity_x*t;
            new_y = velocity_y*t;
            new_z = velocity_z*t;
            
            if new_x>=v2(1) & new_y>=v2(2) & new_z>=v2(3)    
                remove(flyingFLS,i);
                flightPathData(i) = [flightPathData(i);v2];
                arrived = arrived+1;
                timeLook = timeLook+1;
            else
                flyingFLS(i) = [v(1),t+1];
                pnt = [new_x,new_y,new_z];
                flightPathData(i) = [flightPathData(i);pnt];
                flyFLS = [flyFLS;[new_x,new_y,new_z]];
            end
        end
        %disp(flyFLS(1,:));
        for k=1:size(flyFLS,1)
            P1 = flyFLS(k,:);
            for l=k:size(flyFLS,1)
                P2 = flyFLS(l,:);
                P = [P1;P2];
                d = pdist(P,'euclidean');
                comparisons = comparisons+1;
                if d==0
                    
                    collisions = collisions+1;
                end
            end
        end
    end
    for i=1:size(C,1)
        flag = 0;
        val_size = 0;
        if noOfFls(i)<=0
            flag=-1;
        elseif noOfFls(i)<10 & noOfFls(i)>0
            val_size = noOfFls(i);
            noOfFls(i) = noOfFls(i) - noOfFls(i);
            flag=1;
            
        elseif noOfFls(i)>10
            noOfFls(i) = noOfFls(i) - 10;
            flag=0;
        end

        
        keys = distanceDispatcher(i).keys;
        keys = horzcat(keys{:});
        values = distanceDispatcher(i).values;
        values = horzcat(values{:});
        [sortedValues, sortIdx] = sort(values,'descend');
        sortedKeys = keys(sortIdx);
        if flag == 0
            a = sortedKeys(10*cnt+1:10*(cnt+1));
            for j=1:dispatcherRate
                flyingFLS(a(j)) = [i,1];
                
            end
            
        elseif flag == 1
           a = sortedKeys(10*cnt+1:10*cnt+val_size);
           for j=1:val_size
               flyingFLS(a(j)) = [i,1];
           end 
           flag=0;
        elseif flag == -1
            flag=0;
        end

        
      
       
    end    
    cnt = cnt+1; 
    %pause(1);

end
m=max(points);
outputT= ['Number of vertices = ', num2str(size(vertices,1))];
disp(outputT);
displaytime = m/dispatcherRate;
outputT= ['Display Time = ', num2str(displaytime), ' Seconds'];
disp(outputT);
outputT= ['Total distance = ',num2str(dist), ' Cells'];
disp(outputT);
outputT= ['Number of Collisions = ',num2str(collisions)];
disp(outputT);
outputT= ['Number of Comparisons = ',num2str(comparisons)];
disp(outputT);
outputT= ['FLS speed = ',num2str(flsSpeed), ' Cells/Second'];
disp(outputT);
var5 = [];
timeLook=0;
cnt=0;
figure;
while timeLook<size(vertices,1)
    for i=1:size(C,1)
        flag = 0;
        val_size = 0;
        if noOfFls1(i)<=0
            flag=-1;
        elseif noOfFls1(i)<10 & noOfFls1(i)>0
            val_size = noOfFls1(i);
            noOfFls1(i) = noOfFls1(i) - noOfFls1(i);
            flag=1;
            
        elseif noOfFls1(i)>10
            noOfFls1(i) = noOfFls1(i) - 10;
            flag=0;
        end

        
        keys = distanceDispatcher(i).keys;
        keys = horzcat(keys{:});
        values = distanceDispatcher(i).values;
        values = horzcat(values{:});
        [sortedValues, sortIdx] = sort(values,'descend');
        sortedKeys = keys(sortIdx);
        if flag == 0
            a = sortedKeys(10*cnt+1:10*(cnt+1));
            for j=1:dispatcherRate
                timeLook = timeLook+1;
                temp2 = a(j);
                var5 = flightPathData(temp2);
                pause(0.1);
                for k=1:size(var5,1)
                    if k< size(var5,1)
                        plot3(var5(:,1),var5(:,2),var5(:,3), 'Marker','o','Color','c',MarkerIndices=k,LineStyle='--',MarkerEdgeColor='r');
                        hold on;
                    elseif k == size(var5,1)
                        plot3(var5(:,1),var5(:,2),var5(:,3), 'Marker','o','Color','c',MarkerIndices=k,LineStyle='--',MarkerEdgeColor='k',MarkerFaceColor='b');
                        hold on;
                    end
                    
                
                end

            end
            
        elseif flag == 1
           a = sortedKeys(10*cnt+1:10*cnt+val_size);
           for j=1:val_size
               timeLook = timeLook+1;
               temp2 = a(j);
               var5 = flightPathData(temp2);
               pause(0.1);
                for k=1:size(var5,1)
                    if k< size(var5,1)
                        plot3(var5(:,1),var5(:,2),var5(:,3), 'Marker','o','Color','c',MarkerIndices=k,LineStyle='--',MarkerEdgeColor='r');
                        hold on;
                    elseif k == size(var5,1)
                        plot3(var5(:,1),var5(:,2),var5(:,3), 'Marker','o','Color','c',MarkerIndices=k,LineStyle='--',MarkerEdgeColor='k',MarkerFaceColor='b');
                        hold on;
                    end
                    
                
                end

           end 
           flag=0;
        elseif flag == -1
            flag=0;
        end
       
    end    
    cnt = cnt+1; 
end
%{
for i=flightPathData.keys
    var5 = flightPathData(cell2mat(i));
    pause(0.01);
    for j=1:size(var5,1)
        if j< size(var5,1)
            plot3(var5(:,1),var5(:,2),var5(:,3), 'Marker','o','Color','c',MarkerIndices=j,LineStyle='--',MarkerEdgeColor='r');
            hold on;
        elseif j == size(var5,1)
            plot3(var5(:,1),var5(:,2),var5(:,3), 'Marker','o','Color','c',MarkerIndices=j,LineStyle='--',MarkerEdgeColor='k',MarkerFaceColor='b');
            hold on;
        end
        
    
    end

end
%}
hold off;
title('Motion animation');

%{
figure;
plot3(vertices(idx==1,1),vertices(idx==1,2), vertices(idx==1,3),'MarkerSize',12)
hold on
plot3(vertices(idx==2,1),vertices(idx==2,2),vertices(idx==2,3), 'MarkerSize',12)
hold on
plot3(vertices(idx==3,1),vertices(idx==3,2),vertices(idx==3,3), 'MarkerSize',12)
hold on
plot3(vertices(idx==4,1),vertices(idx==4,2),vertices(idx==4,3), 'MarkerSize',12)
hold on
plot3(vertices(idx==5,1),vertices(idx==5,2),vertices(idx==5,3), 'MarkerSize',12)
hold on
plot3(vertices(idx==6,1),vertices(idx==6,2),vertices(idx==6,3), 'MarkerSize',12)
hold on
plot3(vertices(idx==7,1),vertices(idx==7,2),vertices(idx==7,3), 'MarkerSize',12)
hold on
plot3(vertices(idx==8,1),vertices(idx==8,2),vertices(idx==8,3), 'MarkerSize',12)

plot3(dispatchers(:,1),dispatchers(:,2), dispatchers(:,3),'kx','MarkerSize',15,'LineWidth',3,'Color','b') 
legend('Cluster 1','Cluster 2','Cluster 3','Cluster 4','Cluster 5','Cluster 6','Cluster 7','Cluster 8','Dispatchers', 'Location','NW')
title 'Cluster Assignments and Centroids'
hold off
%}








end