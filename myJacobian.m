function J = myJacobian( dx, dq )

    J = sym( 'J', [length(dx), length(dq)]);

    for i = 1 : length( dx )                                               % Iterating along each x,y,z component
        for j = 1 : length( dq )                                           % Iterating along each joint number

            [ tmpc, tmpt ] = coeffs( dx( i ), dq( j ) );                   % Extracting all the coefficients and its corresponding terms

            % IF the coefficients (tmpc) corresponding to dq(3) is empty, put zero
            tmp = tmpc( tmpt == dq( j ) );
            if( isempty( tmp ) )
                J( i, j ) = 0; 
            else    
                J( i, j ) = tmp;
            end

        end
    end

end

