FROM python:3.10
RUN pip install pytest
ADD scripts /scripts
WORKDIR /scripts
CMD ["/usr/local/bin/pytest"]