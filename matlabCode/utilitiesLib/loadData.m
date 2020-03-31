function [handData, fCoords, allData] = loadData(filename)
%loadData loads data
%   Loads data and returns the whole file, the useful hand data and
%   average focal point coordinates 

    allData = load(filename);
    allData(:,end) = [];
    
    [rows,cols] = size(allData);
    
    for j = 1:rows
        for k = 1:cols
            if isnan(allData(j,k))
                error("Incorrectly formatted file")
                return;
            end
        end
    end
    
    handData = [allData(:,7), allData(:,8), allData(:,9)];
    
    k = 1;
    idx = [];
    for j = 1:rows
        if handData(j,3) < -0.5
            idx(k) = j;
            k = k + 1;
        end
    end
    
    if length(idx) ~= 0
        allData(idx,:) = [];
        handData(idx,:) = [];
    end
    
    fCoords = mean([allData(:,4), allData(:,5), allData(:,6)]);    
end 

