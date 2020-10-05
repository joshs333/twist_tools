# Twist Tools
An assortment of tools useful for interacting with twist messages. Mostly a collection of quick utils for me to use.

This package contains the following nodes:
- twist_mux_node (muxes between regular twist messages)
- twist_stamped_mux_node (muxes between stamped twist messages)
- twist_stamper_node (stamps regular twist messages and republishes them)
- twist_unstamper_node (unstamps stamped twist messages and republishes them)

All of the above have example launch files in `launch/`.

Something more generally interesting and useful is the `include/msg_mux/msg_mux.hpp` file which provides a templates mux for topics of any type.

Feel free to ping me for any questions or issues should you find this useful or relevant to you.

**joshs333@live.com**