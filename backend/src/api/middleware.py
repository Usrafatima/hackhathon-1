from fastapi import Request, status
from fastapi.responses import JSONResponse

async def exception_handler(request: Request, call_next):
    try:
        return await call_next(request)
    except Exception as e:
        # Log the exception for debugging purposes
        print(f"Unhandled error: {e}")
        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={"detail": "Sorry, I'm having trouble connecting right now. Please try again in a moment."}
        )