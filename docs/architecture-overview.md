# Architecture Overview

## System Architecture

The Physical AI & Humanoid Robotics Platform is built as a modern, scalable web application with a clear separation of concerns between frontend and backend services. The architecture follows a microservices approach with an API Gateway pattern.

```
┌─────────────────┐    ┌─────────────────────┐    ┌──────────────────┐
│   Frontend      │◄──►│   API Gateway       │◄──►│    Authentication│
│   (Next.js)     │    │   (FastAPI)         │    │    (JWT)         │
└─────────────────┘    └─────────────────────┘    └──────────────────┘
                                │
        ┌────────────────────────┼────────────────────────┐
        │                       │                        │
┌───────▼────────┐    ┌─────────▼─────────┐    ┌───────▼─────────────┐
│   Database     │    │   Vector Store    │    │  Translation &     │
│   (PostgreSQL) │    │   (Qdrant)        │    │  Personalization   │
└────────────────┘    └───────────────────┘    └─────────────────────┘
        │                       │                        │
        └───────────────────────┼────────────────────────┘
                                │
                        ┌───────▼─────────────┐
                        │   AI Processing     │
                        │   (OpenAI API)      │
                        └─────────────────────┘
```

## Component Breakdown

### Frontend Architecture

**Technology Stack:**
- **Framework:** Next.js 15 (App Router)
- **Language:** TypeScript
- **Styling:** Tailwind CSS
- **State Management:** React Context API / Zustand
- **API Communication:** Axios / Fetch API

**Key Components:**
- **Chapter Viewer:** Interactive reader with search capabilities
- **Chat Interface:** Real-time chatbot UI for Q&A functionality
- **Authentication Forms:** Registration, login, and profile management
- **Personalization Controls:** UI elements for content adaptation
- **Translation Toggle:** Switch between English and Urdu content

**Directory Structure:**
```
src/
├── app/                    # Next.js App Router pages
│   ├── auth/              # Authentication pages
│   ├── chat/              # Chatbot interface
│   ├── book/              # Book content pages
│   └── profile/           # User profile management
├── components/            # Reusable React components
├── lib/                   # Utility functions and API calls
├── hooks/                 # Custom React hooks
└── types/                 # TypeScript type definitions
```

### Backend Architecture

**Technology Stack:**
- **Framework:** FastAPI
- **Database:** PostgreSQL with SQLAlchemy ORM
- **Vector Database:** Qdrant Cloud
- **Authentication:** JWT-based with secure password hashing
- **AI Integration:** OpenAI API

**Core Services:**

1. **Authentication Service**
   - User registration/login
   - JWT token generation and validation
   - Profile management

2. **Content Service**
   - Book content management (CRUD operations)
   - Content ingestion and processing
   - Content metadata management

3. **RAG (Retrieval-Augmented Generation) Service**
   - Vector storage and retrieval
   - Semantic search capabilities
   - Content chunking and embedding

4. **AI Service**
   - Chatbot functionality
   - Content summarization
   - Exercise generation
   - Concept explanation

5. **Personalization Service**
   - Content adaptation based on user profile
   - Difficulty level customization
   - Hardware access context

6. **Translation Service**
   - Urdu translation with technical term preservation
   - Translation caching

### API Gateway & Routing

The backend follows a RESTful API architecture with versioning:

```
/api/v1/
├── /auth/                 # Authentication endpoints
├── /chat/                 # Chatbot functionality
├── /content/              # Book content operations
├── /search/               # Search functionality
└── /agent-skills/         # AI-powered features
```

### Database Architecture

**PostgreSQL Schema:**

1. **User Management:**
   - `users`: Core user information (email, password hash, profile)
   - `user_profiles`: Detailed user profiles (software background, hardware access)
   - `user_preferences`: User preferences (language, difficulty level)

2. **Content Management:**
   - `book_content`: Book chapters and content
   - `cached_translations`: Cached Urdu translations
   - `personalized_content_cache`: Personalized content cache

3. **Chat & AI Services:**
   - `chat_sessions`: Chat session management
   - `chat_messages`: Individual messages in chat sessions
   - `agent_skill_executions`: Logging of AI agent executions

4. **Analytics & Tracking:**
   - `search_queries_log`: Search query history and analytics
   - `user_progress`: User progress tracking

### RAG Integration Architecture

**Qdrant Vector Store Schema:**
- **Collection:** `book_content_chunks`
- **Vector Size:** 384 (all-MiniLM-L6-v2 model)
- **Payload Structure:**
  - `chunk_id`: Unique identifier for the chunk
  - `chapter_id`: Reference to the parent chapter
  - `content`: The actual content chunk
  - `section_title`: Title of the section
  - `technical_terms`: List of technical terms in the chunk
  - `difficulty_level`: Difficulty level of the content
  - `tags`: Content tags for filtering

**Embedding Process:**
1. Content is chunked into smaller segments
2. Each chunk is converted to a vector using sentence transformers
3. Vectors are stored in Qdrant with rich metadata
4. Semantic search retrieves relevant content based on user queries

### Caching Strategy

**Database Caching:**
- Personalized content caching with TTL
- Urdu translation caching with invalidation
- Search result caching

**Application Caching:**
- LLM response caching
- Frequently accessed content caching

### Authentication Architecture

**JWT-Based Authentication Flow:**
1. User registers/logs in via `/auth/login` endpoint
2. Backend validates credentials and generates JWT tokens
3. Access token used for API authentication
4. Refresh token for token renewal

**Token Structure:**
- Access Token: 30-minute expiration
- Refresh Token: Long-lived (with automatic renewal)
- Claims: User ID, email, roles

### Security Architecture

**API Security:**
- JWT token validation on protected endpoints
- Rate limiting for API endpoints
- Input validation and sanitization
- SQL injection prevention via ORM
- XSS protection through proper output encoding

**Data Security:**
- Passwords hashed with bcrypt
- Secure connection to databases
- Environment-based configuration
- Sensitive data encryption

### Deployment Architecture

**Frontend Deployment:**
- Static site generation with Next.js
- CDN for asset delivery
- Client-side routing

**Backend Deployment:**
- Containerized deployment (Docker)
- Load balancing
- Health checks and monitoring
- Auto-scaling based on demand

**Database Deployment:**
- Managed PostgreSQL service (Neon)
- Connection pooling
- Backup and recovery

**Vector Database Deployment:**
- Qdrant Cloud with API key authentication
- Separate environment for staging/production
- Performance optimization

## API Contract Overview

### Authentication Endpoints

```
POST /api/v1/auth/register
- Request: { email, password, name, profile }
- Response: { access_token, refresh_token, token_type }

POST /api/v1/auth/login
- Request: { email, password }
- Response: { access_token, refresh_token, token_type }

GET /api/v1/auth/profile
- Headers: { Authorization: Bearer <token> }
- Response: User profile information

PUT /api/v1/auth/profile
- Headers: { Authorization: Bearer <token> }
- Request: Profile update data
- Response: Updated profile information
```

### Chatbot Endpoints

```
POST /api/v1/chat/session
- Headers: { Authorization: Bearer <token> }
- Request: { title, context }
- Response: New chat session

POST /api/v1/chat/{session_id}/messages
- Headers: { Authorization: Bearer <token> }
- Request: { message, context, temperature }
- Response: { session_id, message: ChatMessage }

GET /api/v1/chat/{session_id}/history
- Headers: { Authorization: Bearer <token> }
- Response: [ChatMessage]
```

### Content Endpoints

```
GET /api/v1/content/{chapter_id}/metadata
- Headers: { Authorization: Bearer <token> }
- Response: Chapter metadata

GET /api/v1/content/{chapter_id}/personalized
- Headers: { Authorization: Bearer <token> }
- Request: { difficulty_level, hardware_context }
- Response: Personalized content

GET /api/v1/content/{chapter_id}/urdu
- Headers: { Authorization: Bearer <token> }
- Response: Urdu translation
```

### Search Endpoints

```
POST /api/v1/search/
- Headers: { Authorization: Bearer <token> }
- Request: { query, filters, limit }
- Response: Search results

GET /api/v1/search/chapter/{chapter_id}
- Headers: { Authorization: Bearer <token> }
- Query: { query }
- Response: Chapter-specific search results
```

### Agent Skills Endpoints

```
POST /api/v1/agent-skills/summarize
- Headers: { Authorization: Bearer <token> }
- Request: { chapter_id, target_audience, max_length }
- Response: Summarized content

POST /api/v1/agent-skills/exercises
- Headers: { Authorization: Bearer <token> }
- Request: { chapter_id, exercise_type, target_audience, count }
- Response: Exercise list

POST /api/v1/agent-skills/explain
- Headers: { Authorization: Bearer <token> }
- Request: { concept, target_audience, include_examples }
- Response: Concept explanation
```

## Data Flow Architecture

1. **Content Ingestion:**
   - Markdown content → Content parsing → Database storage → Vector embedding → Qdrant storage

2. **Q&A Flow:**
   - User query → Vector embedding → Semantic search in Qdrant → Content retrieval → LLM processing → Response generation

3. **Personalization Flow:**
   - User profile → Content adaptation rules → Personalized content generation → Caching

4. **Translation Flow:**
   - English content → Translation service → Urdu content → Caching for future use

## Performance & Scalability

**Caching Layers:**
- Application-level caching (Redis)
- Database query caching
- CDN for static assets
- Browser caching headers

**Database Optimization:**
- Connection pooling
- Query optimization
- Indexing strategies
- Read replicas

**AI Service Optimization:**
- Prompt caching
- Response streaming
- Token usage tracking
- Rate limiting

This architecture provides a scalable, maintainable foundation for the Physical AI & Humanoid Robotics educational platform.