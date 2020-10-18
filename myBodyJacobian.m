function JBody = myBodyJacobian( w, v, dq )

JBody = sym('JBody',[length(w) * 2, length(dq)]);
        
for i = 1 : length( w )
   for j = 1 : length( dq )
        
        [tmpwc,tmpwt] = coeffs( w(i), dq(j) );    % Extracting all the coefficients and its corresponding terms
        [tmpvc,tmpvt] = coeffs( v(i), dq(j) );    % Extracting all the coefficients and its corresponding terms

        
        % IF the coefficients (tmpc) corresponding to dq(3) is empty, put zero
        if( isempty( tmpvc( tmpvt == dq(j) ) ) )
            JBody(i,j) = 0; 
        else    
            JBody(i,j) = tmpvc( tmpvt == dq(j) );
        end
        
        % IF the coefficients (tmpc) corresponding to dq(3) is empty, put zero
        if( isempty( tmpwc( tmpwt == dq(j) ) ) )
            JBody(i+3,j) = 0; 
        else    
            JBody(i+3,j) = tmpwc( tmpwt == dq(j) );
        end
        
       
   end     
end
 


end

