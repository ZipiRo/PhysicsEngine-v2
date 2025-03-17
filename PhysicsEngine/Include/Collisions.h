#pragma once

void PointSegmentDistance(Vector2D point, Vector2D a, Vector2D b, 
    float& distanceSquared, Vector2D& closestPoint)
{
    Vector2D AB = b - a;
    Vector2D AP = point - a;

    float projection = Vector2D::DotProduct(AP, AB);
    float ABLengthSquared = VectorMath::LengthSquared(AB);
    float t = projection / ABLengthSquared;

    if(t <= 0)
    {
        closestPoint = a;
    }
    else if(t >= 1.0f)
    {
        closestPoint = b;
    }
    else
    {
        closestPoint = a + AB * t;
    }

    distanceSquared = VectorMath::DistanceSquared(point, closestPoint);
}
void ProjectVertices(std::vector<Vector2D>vertices, Vector2D axis, float& min, float& max)
{
    min = infinity;
    max = -infinity;

    for (Vector2D vertex : vertices)
    {
        float projection = Vector2D::DotProduct(vertex, axis);
        
        if(projection < min) min = projection;
        if(projection > max) max = projection;   
    }
}
void ProjectCircle(Vector2D center, float radius, Vector2D axis, float& min, float& max)
{
    Vector2D direction = Vector2D::Normalize(axis);
    Vector2D directionRadius = direction * radius;

    Vector2D point1 = center + directionRadius;
    Vector2D point2 = center - directionRadius;

    min = Vector2D::DotProduct(point1, axis);
    max = Vector2D::DotProduct(point2, axis);

    if(min > max)
    {
        float aux = min;
        min = max;
        max = aux;
    }
}
int ClosestPointOnPolygon(Vector2D center, std::vector<Vector2D> vertices)
{
    int result = -1, i = 0;
    float minDistance = infinity;


    for(Vector2D vertex : vertices)
    {
        i++;
        float distance = Vector2D::Distance(vertex, center);

        if(distance < minDistance)
        {
            minDistance = distance;
            result = i;
        }
    }      

    return result;
}

void FindCircleContactPoint(Vector2D centerA, float radiusA, Vector2D centerB, 
    Vector2D& contactPoint)
{
    contactPoint = Vector2D(0, 0);

    Vector2D AB = centerB - centerA;
    Vector2D direction = Vector2D::Normalize(AB);

    contactPoint = centerA + direction * radiusA;
}

void FindCirclePolygonContactPoint(Vector2D circleCenter, float radius, std::vector<Vector2D>vertices, Vector2D polygonCenter, 
    Vector2D& contactPoint)
{   
    contactPoint = Vector2D(0, 0);
    float min_distanceSquared = infinity;

    for(auto vertex = vertices.begin(); vertex != vertices.end(); ++vertex)
    {
        auto next_vertex = std::next(vertex);
        if(next_vertex == vertices.end())
            next_vertex = vertices.begin(); 

        Vector2D vertexA = *vertex;
        Vector2D vertexB = *next_vertex;

        float distanceSquared;
        Vector2D closestPoint;
        PointSegmentDistance(circleCenter, vertexA, vertexB, distanceSquared, closestPoint);

        if(distanceSquared < min_distanceSquared)
        {
            min_distanceSquared = distanceSquared;
            contactPoint = closestPoint;
        }
    }
}

void FindPolygonsContactPoints(std::vector<Vector2D>verticesA, std::vector<Vector2D>verticesB, 
    Vector2D& contactOne, Vector2D& contactTwo, int& contactCount)
{
    contactOne = Vector2D(0, 0);
    contactTwo = Vector2D(0, 0);
    contactCount = 0;

    float min_distanceSquared = infinity;

    for(auto vertex_it = verticesA.begin(); vertex_it != verticesA.end(); ++vertex_it)
    {
        Vector2D point = *vertex_it;

        for(auto vertex_jt = verticesB.begin(); vertex_jt != verticesB.end(); ++vertex_jt)
        {
            auto next_vertex_jt = std::next(vertex_jt);
            if(next_vertex_jt == verticesB.end())
                next_vertex_jt = verticesB.begin(); 

            Vector2D vertexA = *vertex_jt;
            Vector2D vertexB = *next_vertex_jt;

            float distanceSquared;
            Vector2D closestPoint;
            PointSegmentDistance(point, vertexA, vertexB, distanceSquared, closestPoint);
            
            if(NearlyEqual(distanceSquared, min_distanceSquared))
            {
                if(!VectorMath::NearlyEqualVectors(closestPoint, contactOne))
                {
                    contactTwo = closestPoint;
                    contactCount = 2;
                }
            }
            else if(distanceSquared < min_distanceSquared)
            {
                min_distanceSquared = distanceSquared;
                contactOne = closestPoint;
                contactCount = 1;
            }
        }
    }
    
    for(auto vertex_it = verticesB.begin(); vertex_it != verticesB.end(); ++vertex_it)
    {
        Vector2D point = *vertex_it;

        for(auto vertex_jt = verticesA.begin(); vertex_jt != verticesA.end(); ++vertex_jt)
        {
            auto next_vertex_jt = std::next(vertex_jt);
            if(next_vertex_jt == verticesA.end())
                next_vertex_jt = verticesA.begin(); 

            Vector2D vertexA = *vertex_jt;
            Vector2D vertexB = *next_vertex_jt;

            float distanceSquared;
            Vector2D closestPoint;
            PointSegmentDistance(point, vertexA, vertexB, distanceSquared, closestPoint);
            
            if(NearlyEqual(distanceSquared, min_distanceSquared))
            {
                if(!VectorMath::NearlyEqualVectors(closestPoint, contactOne))
                {
                    contactTwo = closestPoint;
                    contactCount = 2;
                }
            }
            else if(distanceSquared < min_distanceSquared)
            {
                min_distanceSquared = distanceSquared;
                contactOne = closestPoint;
                contactCount = 1;
            }
        }
    }
}

bool IntersectAABB(AABB a, AABB b) {
    if (a.max.x <= b.min.x || b.max.x <= a.min.x) return false;
    if (a.max.y <= b.min.y || b.max.y <= a.min.y) return false;

    return true;
}

bool IntersectCircles(Vector2D centerA, float radiusA, Vector2D centerB, float radiusB, 
    Vector2D &normal, float &depth) 
{
    normal = Vector2D(0, 0);
    depth = 0.0f;

    float distance = Vector2D::Distance(centerA, centerB);
    float radiI = radiusA + radiusB;

    if(distance >= radiI)
    {
        return false;
    }

    normal = centerB - centerA;
    normal = Vector2D::Normalize(normal);
    depth = radiI - distance;

    return true;
}

bool IntersectPolygons(std::vector<Vector2D>verticesA, Vector2D centerA, std::vector<Vector2D>verticesB, Vector2D centerB, 
    Vector2D &normal, float &depth)
{
    normal = Vector2D(0, 0);
    depth = infinity;
    
    Vector2D axis = Vector2D(0, 0);
    float minA, minB, maxA, maxB;
    
    for(auto vertex = verticesA.begin(); vertex != verticesA.end(); ++vertex)
    {
        auto next_vertex = std::next(vertex);
        if(next_vertex == verticesA.end())
            next_vertex = verticesA.begin(); 

        Vector2D vertexA = *vertex;
        Vector2D vertexB = *next_vertex;

        Vector2D edge = vertexB - vertexA;
        axis = Vector2D(edge.y, -edge.x);
        axis = Vector2D::Normalize(axis);

        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        if(minA >= maxB || minB >= maxA)
        {
            return false;
        }

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if(axisDepth < depth) 
        {
            depth = axisDepth;
            normal = axis;
        }
    }

    for(auto vertex = verticesB.begin(); vertex != verticesB.end(); ++vertex)
    {
        auto next_vertex = std::next(vertex);
        if(next_vertex == verticesA.end())
            next_vertex = verticesA.begin(); 

        Vector2D vertexA = *vertex;
        Vector2D vertexB = *next_vertex;

        Vector2D edge = vertexB - vertexA;
        axis = Vector2D(edge.y, -edge.x);
        axis = Vector2D::Normalize(axis);

        ProjectVertices(verticesA, axis, minA, maxA);
        ProjectVertices(verticesB, axis, minB, maxB);

        if(minA >= maxB || minB >= maxA)
        {
            return false;
        }

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if(axisDepth < depth) 
        {
            depth = axisDepth;
            normal = axis;
        }
    }

    Vector2D direction = centerB - centerA;

    if(Vector2D::DotProduct(direction, normal) < 0.0f)
    {
        normal = -normal;
    }

    return true;
}

bool IntersectCirclesPolygons(Vector2D circleCenter, float radius, std::vector<Vector2D>vertices, Vector2D polygonCenter,
    Vector2D &normal, float &depth)
{
    normal = Vector2D(0, 0);
    depth = infinity;

    Vector2D axis = Vector2D(0, 0);
    float minA, minB, maxA, maxB;

    for(auto vertex = vertices.begin(); vertex != vertices.end(); ++vertex)
    {
        
        auto next_vertex = std::next(vertex);
        if(next_vertex == vertices.end())
            next_vertex = vertices.begin(); 

        Vector2D vertexA = *vertex;
        Vector2D vertexB = *next_vertex;

        Vector2D edge = vertexB - vertexA;
        axis = Vector2D(edge.y, -edge.x);
        axis = Vector2D::Normalize(axis);

        ProjectVertices(vertices, axis, minA, maxA);
        ProjectCircle(circleCenter, radius, axis, minB, maxB);

        if(minA >= maxB || minB >= maxA)
        {
            return false;
        }

        float axisDepth = std::min(maxB - minA, maxA - minB);

        if(axisDepth < depth) 
        {
            depth = axisDepth;
            normal = axis;
        }
    }

    int closestPoint_Index = ClosestPointOnPolygon(circleCenter, vertices);

    int index = 0; Vector2D closestPoint;
    for(Vector2D v : vertices)
        if(closestPoint_Index == ++index) { closestPoint = v; break; }

    axis = closestPoint - circleCenter;
    axis = Vector2D::Normalize(axis);

    ProjectVertices(vertices, axis, minA, maxA);
    ProjectCircle(circleCenter, radius, axis, minB, maxB);

    if(minA >= maxB || minB >= maxA)
    {
        return false;
    }

    float axisDepth = std::min(maxB - minA, maxA - minB);

    if(axisDepth < depth) 
    {
        depth = axisDepth;
        normal = axis;
    }

    Vector2D direction = polygonCenter - circleCenter;

    if(Vector2D::DotProduct(direction, normal) < 0.0f)
    {
        normal = -normal;
    }

    return true;
}