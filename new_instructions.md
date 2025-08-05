# Instructions

## Copy connection script to SAMA board

On your laptop, do the following steps.

1. Open a terminal
2. cd ~/aiml2/microchip-masters
3. scp connect_edge.osql root@10.0.0.20:/root/masters/.

Now you should have the connection script on your SAMA board.


## Start local nameserver

On your laptop, do the following steps.

1. cd ~/aiml2/microchip-masters/sa.engine/bin/
2. ./sa.engine -n local_nameserver

You now have a SA Engine server running on your laptop. Leave the terminal open.


## Register local client with local nameserver

On your laptop, in VSCode intercative window, run the following OSQL command.

1. reregister("client_laptop");

Your VSCode client is now connected to the local server.


## Connect the edge on SAMA to nameserver on laptop

On the SAMA board, run the following commands.

1. cd ~/masters
2. sa.engine -O connect_edge.osql

SA Engine on the SAMA board should now be connected to the nameserver on the laptop. Leave the terminal open.


## Verify connection

On your laptop, in VSCode interactive window, run the following OSQL command.

1. other_peers();

In the result list you should see the name 'Edge00'.



