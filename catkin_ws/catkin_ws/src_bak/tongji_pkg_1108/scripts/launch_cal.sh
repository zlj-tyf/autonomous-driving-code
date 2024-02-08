
{
gnome-terminal  -x bash -c "python3 ./intersc_manage_1031.py ;exec bash"
}&
sleep 2s
{
gnome-terminal  -x bash -c "python3 ./compute_node_1_test4cam.py ;exec bash"
}&
sleep 2s
{
gnome-terminal  -x bash -c "python3 ./compute_node_2_test4cam.py ;exec bash"
}&
sleep 2s
{
gnome-terminal  -x bash -c "python3 ./compute_node_3_test4cam.py ;exec bash"
}&
sleep 2s
{
gnome-terminal  -x bash -c "python3 ./compute_node_4_test4cam.py ;exec bash"
}&
sleep 1s
