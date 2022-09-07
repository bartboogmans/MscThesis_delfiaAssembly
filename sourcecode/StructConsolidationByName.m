%% Consolidate the substructures into one struct.
%   DO NOT EDIT the code of this section    
    
    sas = whos();
    sas( not( strcmp({sas.class},'struct') ) ) = [];
    %
    for sub = {sas.name}
        options.( sub{:} ) = eval( sub{:} );
    end
    
    
    