function exp_array=expand_array(node_x,node_y,hn,xTarget,yTarget,CLOSED,MAX_X,MAX_Y)
    exp_array=[];
    exp_count=1;
    c2=size(CLOSED,1);
    for k= 1:-1:-1
        for j= 1:-1:-1
            if k~=j%正上下左右方向搜索   
                s_x = node_x+k;
                s_y = node_y+j;
                if s_x >0 
                    if s_x <=MAX_X
                        if s_y >0 
                            if s_y <=MAX_Y
                                flag=1;%搜索范围内                    
                                for c1=1:c2
                                    if s_x == CLOSED(c1,1) 
                                        if s_y == CLOSED(c1,2)
                                            flag=0;%(s_x,s_y)是障碍物
                                        end
                                    end
                                end
                                if (flag == 1)%不是障碍物可以作为下一点
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
			else
				if k~=0%排除自身，斜搜索
                    s_x = node_x+k;
                    s_y = node_y+j;
                    if s_x >0 
                        if s_x <=MAX_X
                            if s_y >0 
                                if s_y <=MAX_Y
                                    flag=1;                    
                                    for c1=1:c2
                                        if s_x == CLOSED(c1,1) 
                                            if s_y == CLOSED(c1,2)
                                                flag=0;
                                            end
                                        end
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
            end
        end
    end  
end