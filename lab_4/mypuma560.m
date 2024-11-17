function [robot] = mypuma560(DH)
    % Given a 6x6 DH matrix, create a SerialLink object 
    % (https://www.petercorke.com/RTB/r9/html/SerialLink.html)
    links = make_links(DH);
    robot = SerialLink(links);
end


function [link_list] = make_links(DH)
    % Given a 6x6 DH matrix, create a list of Link objects
    % (https://www.petercorke.com/RTB/r9/html/Link.html)
    for i = 1:6
        % theta values in the DH matrix are not used here since all links
        % are revolute.
        link_list(i) = Link('a', DH(i,1),'alpha', DH(i,2), 'd', DH(i,3));
    end
end

