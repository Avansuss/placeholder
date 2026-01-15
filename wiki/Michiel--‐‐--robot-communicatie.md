
Het idee is dat iedere robot luistert met een node op een bepaalde topic. Hierop krijgt hij data binnen in een string, deze data moet de node parsen naar de juiste waarden. Alle robots kunnen luisteren op hetzelfde topic bijvoorbeeld start_info. Dit komt omdat je op de hoofdcoordinator met het command ```ros2 topic pub /robotNUM/TOPIC std_msgs/String "data: '{DATA'}" ```.
