# Use the official Python image
FROM python:3.9-slim

# Set the working directory
WORKDIR /app

# Copy the app code into the container
COPY . /app

# Install dependencies
RUN pip install Flask

# Expose the port the app will run on
EXPOSE 5000

# Run the application
CMD ["python", "app.py"]
