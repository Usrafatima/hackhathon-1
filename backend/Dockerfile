# Use an official Python runtime as a parent image
FROM python:3.11-slim-buster

# Set the working directory in the container
WORKDIR /app

# Install any needed packages specified in requirements.txt
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the application code
COPY backend/ ./backend

# Expose the port FastAPI runs on
EXPOSE 8000

# Run the FastAPI application
CMD ["python", "-m", "backend.main", "--host", "0.0.0.0", "--port", "8000"]