# With this file you can make the try of using custom images, just be careful as the size of the
# download is big and it may take a bit. Also, when deleting this please consider that you are
# removing a Ubuntu/Debian image where there are processes that will remain until reboot.

# Dir for the container
mkdir xenial

# Install xenial
sudo deboostrap xenial ./xenial

# Check the installation
ls ./xenial

# Expor as an image
sudo tar -C xenial/ -c . docker image import - xenial

# Search for the xenial image
docker image ls