from backend.src.models.user_profile import UserProfile
from typing import Dict, Optional

class UserService:
    def __init__(self):
        self.user_profiles_db: Dict[str, UserProfile] = {}

    def get_user_profile(self, user_id: str) -> Optional[UserProfile]:
        return self.user_profiles_db.get(user_id)

    def create_user_profile(self, user_profile: UserProfile) -> UserProfile:
        if user_profile.id is None:
            # Generate a simple ID if not provided, though Google OAuth provides one
            user_profile.id = str(len(self.user_profiles_db) + 1)
        
        if user_profile.id in self.user_profiles_db:
            raise ValueError("User with this ID already exists")
        if any(p.username == user_profile.username for p in self.user_profiles_db.values()):
            raise ValueError("Username already taken")

        self.user_profiles_db[user_profile.id] = user_profile
        return user_profile

    def update_user_profile(self, user_id: str, user_profile: UserProfile) -> UserProfile:
        if user_id not in self.user_profiles_db:
            raise ValueError("User not found")
        
        if user_profile.id and user_profile.id != user_id:
            raise ValueError("User ID in path and body do not match")

        self.user_profiles_db[user_id] = user_profile
        return self.user_profiles_db[user_id]

    def get_user_profile_by_username(self, username: str) -> Optional[UserProfile]:
        for user_profile in self.user_profiles_db.values():
            if user_profile.username == username:
                return user_profile
        return None

user_service = UserService()
