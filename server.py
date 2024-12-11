from fastapi.middleware.cors import CORSMiddleware
from fastapi import FastAPI
import uvicorn
from visualize import generate_trajectory

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
def read_root():
    return {"message": "Hello, World!"}


@app.post("/trajectory")
async def read_trajectory(v_conveyor=0.3, obj_pos_y=0.2, duration=0.25):
    print(v_conveyor, obj_pos_y, duration)
    try:
        theta_jointvel_set = generate_trajectory(
            float(v_conveyor), float(obj_pos_y), float(duration))
        return {"status": "success", "data": theta_jointvel_set}
    except Exception as e:
        return {"status": "error", "message": str(e)}


if __name__ == "__main__":
    uvicorn.run(app, host="localhost", port=3001)
