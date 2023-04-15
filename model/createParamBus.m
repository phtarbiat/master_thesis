function createParamBus(paramStruct,mainBus,ws)
% Generate Simulink-Bus from model parameter struct
% The struct has 2 levels -> nested struct 
%
% Philipp Tarbiat
% Chair of Automatic Control
% TUM School of Engineering and Design
% Technical University of Munich


% Obtain the fieldnames of the structure
scategories = fieldnames(paramStruct);

for iCat = 1:length(scategories)

    % Obtain the fieldnames of the sub structure
    sfields = fieldnames(paramStruct.(scategories{iCat}));

    % Loop through the structure
    for iField = 1:length(sfields)

        % Create BusElement for each field
        elems(iField) = Simulink.BusElement;
        elems(iField).Name = sfields{iField};
        elems(iField).Dimensions = size(paramStruct.(scategories{iCat}).(sfields{iField}));
        elems(iField).DataType = class(paramStruct.(scategories{iCat}).(sfields{iField}));
        elems(iField).SampleTime = -1;
        elems(iField).Complexity = 'real';
        elems(iField).SamplingMode = 'Sample based';

    end

    % Create sub bus that holds elements
    eval(sprintf('%s = Simulink.Bus;', scategories{iCat}));
    eval(sprintf('%s.Elements = elems;', scategories{iCat}));

    % Assign sub bus to workspace variable
    eval(sprintf('assignin(ws,scategories{iCat},%s);', scategories{iCat}));

    % Create categories of bus elements for the main Bus that corresbond to
    % the sub Busses
    elemsCat(iCat) = Simulink.BusElement;
    elemsCat(iCat).Name = scategories{iCat};
    elemsCat(iCat).DataType = ['Bus: ' scategories{iCat}];

    clear elems

end

% Create main bus
main_bus = Simulink.Bus;
main_bus.Elements = elemsCat;

% Assign main bus to workspace variable
assignin(ws,mainBus,main_bus);