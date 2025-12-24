import streamlit as st
from llama_index.core import SimpleDirectoryReader, VectorStoreIndex, Settings
from llama_index.llms.groq import Groq
from llama_index.embeddings.huggingface import HuggingFaceEmbedding
import os

# Set your Groq API key (add as Streamlit Secret – see below)
GROQ_API_KEY = st.secrets.get("GROQ_API_KEY") or os.getenv("GROQ_API_KEY")

if not GROQ_API_KEY:
    st.error("Please set GROQ_API_KEY in Streamlit Secrets (Manage app → Secrets)")
    st.stop()

@st.cache_resource
def load_index():
    with st.spinner("Indexing your book – this takes 2–5 minutes on first load..."):
        reader = SimpleDirectoryReader(input_dir="docs/", recursive=True, required_exts=[".md"])
        docs = reader.load_data()
        st.info(f"Loaded {len(docs)} chunks from your book")

        # Small & fast embedding model
        embed_model = HuggingFaceEmbedding(model_name="BAAI/bge-small-en-v1.5")
        Settings.embed_model = embed_model

        index = VectorStoreIndex.from_documents(docs, show_progress=True)

        # Groq LLM (fast & free tier)
        llm = Groq(model="llama-3.1-8b-instant", api_key=GROQ_API_KEY, temperature=0.3)
        Settings.llm = llm

        query_engine = index.as_query_engine(streaming=True, similarity_top_k=5)
        return query_engine

query_engine = load_index()

st.title("🤖 Physical AI & Humanoid Robots Book Assistant")
st.caption("Ask anything — answers from your course book only!")

if "messages" not in st.session_state:
    st.session_state.messages = []

for message in st.session_state.messages:
    with st.chat_message(message["role"]):
        st.markdown(message["content"])

if query := st.chat_input("Ask a question about the course..."):
    st.session_state.messages.append({"role": "user", "content": query})
    with st.chat_message("user"):
        st.markdown(query)

    with st.chat_message("assistant"):
        with st.spinner("Thinking..."):
            response = query_engine.query(query)
            st.markdown(str(response))

    st.session_state.messages.append({"role": "assistant", "content": str(response)})