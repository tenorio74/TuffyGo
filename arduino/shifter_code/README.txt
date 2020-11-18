Here is how I believe the gear shifter should work...

Need three wires coming from Jetson
    Wire 1: signal
    Wire 2: forward/backwards 
    Wire 3: park


1. Every time the cart is turned on, the very first thing it will do is re-initialize itself
    and figure out where neutral is.

2. Wait for a signal from the Jetson to tell the shifter that we need to do something
    If the signal is HIGH, do something... if it's low, do nothing...

3. Once it gets the signal, it should look for what gear to change into
    This will come from another signal wire.
    If the signal is HIGH, go forward
    If the signal is LOW, go backward

4. If park is HIGH, go through the startup rootine to get the cart back to neutral. Otherwise ignore it.